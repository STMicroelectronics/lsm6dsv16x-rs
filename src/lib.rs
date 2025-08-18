#![no_std]
#![doc = include_str!("../README.md")]
pub mod prelude;
pub mod register;

#[cfg(feature = "passthrough")]
use core::cell::RefCell;
#[cfg(feature = "passthrough")]
use core::cell::RefMut;
use core::fmt::Debug;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;
use half::f16;
use st_mems_bus::{BusOperation, EmbAdvFunctions, MemBankFunctions, i2c::*};

use prelude::*;

/// Driver for the LSM6DSV16X sensor.
///
/// The struct takes a bus and a timer hardware object to write to the
/// registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
pub struct Lsm6dsv16x<B, T> {
    /// The bus driver.
    pub bus: B,
    pub tim: T,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    /// Incapsulate bus errors
    Bus(B),
    /// Unexpected value read from a register
    UnexpectedValue,
    FailedToReadMemBank,
    FailedToSetMemBank(MemBank),
}

impl<B> From<B> for Error<B>
where
    B: Debug,
{
    fn from(bus_error: B) -> Self {
        Error::Bus(bus_error)
    }
}

impl<P, T> Lsm6dsv16x<I2cBus<P>, T>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = st_mems_bus::i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus, tim }
    }
}
impl<B, T> Lsm6dsv16x<B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    /// Create a sensor instance from a generic bus.
    ///
    /// The Bus must implement the `BusOperation` trait
    pub fn from_bus(bus: B, tim: T) -> Self {
        Self { bus, tim }
    }
}

impl<P, T> Lsm6dsv16x<st_mems_bus::spi::SpiBus<P>, T>
where
    P: SpiDevice,
    T: DelayNs,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = st_mems_bus::spi::SpiBus::new(spi);
        Self { bus, tim }
    }
}

impl<B, T> MemBankFunctions<MemBank> for Lsm6dsv16x<B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    type Error = Error<B::Error>;

    /// Change memory bank.
    fn mem_bank_set(&mut self, val: MemBank) -> Result<(), Self::Error> {
        let mut func_cfg_access =
            FuncCfgAccess::read(self).map_err(|_| Error::FailedToReadMemBank)?;

        func_cfg_access.set_shub_reg_access(((val as u8) & 0x02) >> 1);
        func_cfg_access.set_emb_func_reg_access(val as u8 & 0x01);
        func_cfg_access
            .write(self)
            .map_err(|_| Error::FailedToSetMemBank(val))
    }
    /// Get the memory bank actually set.
    fn mem_bank_get(&mut self) -> Result<MemBank, Self::Error> {
        let func_cfg_access = FuncCfgAccess::read(self).map_err(|_| Error::FailedToReadMemBank)?;
        let value =
            (func_cfg_access.shub_reg_access() << 1) + func_cfg_access.emb_func_reg_access();

        let val = match value {
            0 => MemBank::MainMemBank,
            1 => MemBank::EmbedFuncMemBank,
            2 => MemBank::SensorHubMemBank,
            _ => MemBank::MainMemBank,
        };

        Ok(val)
    }
}

impl<B, T> EmbAdvFunctions for Lsm6dsv16x<B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    type Error = Error<B::Error>;

    /// Write buffer in a page.
    ///
    /// # Arguments
    ///
    /// * `address`: Address where the page write begins.
    /// * `buf`: Buffer to write in a page.
    /// * `len`: Length of the buffer.
    fn ln_pg_write(&mut self, address: u16, buf: &[u8], len: u8) -> Result<(), Self::Error> {
        let mut msb = ((address >> 8) & 0x0F) as u8;
        let mut lsb = (address & 0xFF) as u8;

        MemBank::operate_over_embed(self, |state| {
            // Set page write
            let mut page_rw = PageRw::read(state)?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(1);
            page_rw.write(state)?;

            // Select page
            let mut page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(msb);
            page_sel.write(state)?;

            // Set page address
            let mut page_address = PageAddress::from_bits(0);
            page_address.set_page_addr(lsb);
            page_address.write(state)?;

            for i in 0..len {
                state.write_to_register(
                    EmbReg::PageValue as u8,
                    &buf[i as usize..(i as usize + 1)],
                )?;

                lsb = lsb.wrapping_add(1);
                // Check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(state)?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(state)?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(0);
            page_sel.write(state)?;

            // Unset page write
            page_rw = PageRw::read(state)?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(0);
            page_rw.write(state)
        })
    }

    /// Read buffer in a page.
    ///
    /// # Arguments
    ///
    /// * `address`: The address to read from.
    /// * `buf`: Write buffer in a page.
    /// * `len`: Length of the buffer.
    fn ln_pg_read(&mut self, address: u16, buf: &mut [u8], len: u8) -> Result<(), Self::Error> {
        let mut msb = ((address >> 8) & 0x0F) as u8;
        let mut lsb = (address & 0xFF) as u8;

        MemBank::operate_over_embed(self, |state| {
            // Set page read
            let mut page_rw = PageRw::read(state)?;
            page_rw.set_page_read(1);
            page_rw.set_page_write(0);
            page_rw.write(state)?;

            // Select page
            let mut page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(msb);
            page_sel.write(state)?;

            // Set page address
            let mut page_address = PageAddress::from_bits(0);
            page_address.set_page_addr(lsb);
            page_address.write(state)?;

            for i in 0..len {
                state.read_from_register(
                    EmbReg::PageValue as u8,
                    &mut buf[i as usize..(i as usize + 1)],
                )?;

                lsb = lsb.wrapping_add(1);
                // Check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(state)?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(state)?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(state)?;
            page_sel.set_page_sel(0);
            page_sel.write(state)?;

            // Unset page read
            page_rw = PageRw::read(state)?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(0);
            page_rw.write(state)
        })
    }
}

impl<B: BusOperation, T: DelayNs> Lsm6dsv16x<B, T> {
    pub fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_to_register(reg, buf).map_err(Error::Bus)
    }

    pub fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_from_register(reg, buf).map_err(Error::Bus)
    }
    /// Enables accelerometer user offset correction block.
    ///
    /// It is valid for the low-pass path.
    pub fn xl_offset_on_out_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self)?;
        ctrl9.set_usr_off_on_out(val);
        ctrl9.write(self)?;
        Ok(())
    }
    /// Get accelerometer user offset correction block (enable/disable).
    ///
    /// It is valid for the low-pass path.
    pub fn xl_offset_on_out_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).map(|reg| reg.usr_off_on_out())?;

        Ok(val)
    }
    /// Set the Accelerometer user offset correction values in mg.
    pub fn xl_offset_mg_set(&mut self, val: XlOffsetMg) -> Result<(), Error<B::Error>> {
        let mut z_ofs_usr = ZOfsUsr::read(self)?;
        let mut y_ofs_usr = YOfsUsr::read(self)?;
        let mut x_ofs_usr = XOfsUsr::read(self)?;

        let mut ctrl9 = Ctrl9::read(self)?;

        if (val.x_mg < (0.0078125 * 127.0) && val.x_mg > (0.0078125 * -127.0))
            && (val.y_mg < (0.0078125 * 127.0) && val.y_mg > (0.0078125 * -127.0))
            && (val.z_mg < (0.0078125 * 127.0) && val.z_mg > (0.0078125 * -127.0))
        {
            ctrl9.set_usr_off_w(0);

            let tmp = val.z_mg / 0.0078125;
            z_ofs_usr.set_z_ofs_usr(tmp as u8);

            let tmp = val.y_mg / 0.0078125;
            y_ofs_usr.set_y_ofs_usr(tmp as u8);

            let tmp = val.x_mg / 0.0078125;
            x_ofs_usr.set_x_ofs_usr(tmp as u8);
        } else if (val.x_mg < (0.125 * 127.0) && val.x_mg > (0.125 * -127.0))
            && (val.y_mg < (0.125 * 127.0) && val.y_mg > (0.125 * -127.0))
            && (val.z_mg < (0.125 * 127.0) && val.z_mg > (0.125 * -127.0))
        {
            ctrl9.set_usr_off_w(1);

            let tmp = val.z_mg / 0.125;
            z_ofs_usr.set_z_ofs_usr(tmp as u8);

            let tmp = val.y_mg / 0.125;
            y_ofs_usr.set_y_ofs_usr(tmp as u8);

            let tmp = val.x_mg / 0.125;
            x_ofs_usr.set_x_ofs_usr(tmp as u8);
        } else {
            ctrl9.set_usr_off_w(1);
            z_ofs_usr.set_z_ofs_usr(0xFF);
            y_ofs_usr.set_y_ofs_usr(0xFF);
            x_ofs_usr.set_x_ofs_usr(0xFF);
        }

        z_ofs_usr.write(self)?;
        y_ofs_usr.write(self)?;
        x_ofs_usr.write(self)?;
        ctrl9.write(self)
    }
    /// Get the Accelerometer user offset correction values in mg.
    pub fn xl_offset_mg_get(&mut self) -> Result<XlOffsetMg, Error<B::Error>> {
        let ctrl9 = Ctrl9::read(self)?;
        let z_ofs_usr = ZOfsUsr::read(self)?;
        let y_ofs_usr = YOfsUsr::read(self)?;
        let x_ofs_usr = XOfsUsr::read(self)?;

        let val = XlOffsetMg {
            z_mg: if ctrl9.usr_off_w() == 0 {
                z_ofs_usr.z_ofs_usr() as f32 * 0.0078125
            } else {
                z_ofs_usr.z_ofs_usr() as f32 * 0.125
            },
            y_mg: if ctrl9.usr_off_w() == 0 {
                y_ofs_usr.y_ofs_usr() as f32 * 0.0078125
            } else {
                y_ofs_usr.y_ofs_usr() as f32 * 0.125
            },
            x_mg: if ctrl9.usr_off_w() == 0 {
                x_ofs_usr.x_ofs_usr() as f32 * 0.0078125
            } else {
                x_ofs_usr.x_ofs_usr() as f32 * 0.125
            },
        };

        Ok(val)
    }
    /// Reset of the device.
    pub fn reset_set(&mut self, val: Reset) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self)?;
        let mut func_cfg_access = FuncCfgAccess::read(self)?;

        ctrl3.set_boot(((val as u8) & 0x04) >> 2);
        ctrl3.set_sw_reset(((val as u8) & 0x02) >> 1);
        func_cfg_access.set_sw_por(val as u8 & 0x01);

        ctrl3.write(self)?;
        func_cfg_access.write(self)
    }
    /// Get reset state of the device.
    pub fn reset_get(&mut self) -> Result<Reset, Error<B::Error>> {
        let ctrl3 = Ctrl3::read(self)?;
        let func_cfg_access = FuncCfgAccess::read(self)?;

        let value: u8 = (ctrl3.sw_reset() << 2) + (ctrl3.boot() << 1) + func_cfg_access.sw_por();
        let val = Reset::try_from(value).unwrap_or(Reset::GlobalRst);

        Ok(val)
    }

    /// Get Device ID.
    ///
    /// This function works also for OIS
    /// (WHO_AM_I and SPI2_WHO_AM_I have same address).
    pub fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|reg| reg.into())
    }
    /// Set Accelerometer output data rate (ODR).
    pub fn xl_data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut ctrl1 = Ctrl1::read(self)?;

        ctrl1.set_odr_xl(val as u8 & 0x0F);
        ctrl1.write(self)?;

        let sel = (val as u8 >> 4) & 0x0F;
        if sel != 0 {
            let mut haodr = HaodrCfg::read(self)?;

            haodr.set_haodr_sel(sel);
            haodr.write(self)?;
        }

        Ok(())
    }
    /// Get accelerometer output data rate (ODR).
    pub fn xl_data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let ctrl1 = Ctrl1::read(self)?;
        let haodr = HaodrCfg::read(self)?;
        let sel = haodr.haodr_sel() << 4;
        let odr = sel | ctrl1.odr_xl();

        Ok(Odr::try_from(odr).unwrap_or_default())
    }
    /// Set accelerometer operating mode.
    pub fn xl_mode_set(&mut self, val: XlMode) -> Result<(), Error<B::Error>> {
        let mut ctrl1 = Ctrl1::read(self)?;
        ctrl1.set_op_mode_xl((val as u8) & 0x07);
        ctrl1.write(self)
    }
    /// Get accelerometer operating mode.
    pub fn xl_mode_get(&mut self) -> Result<XlMode, Error<B::Error>> {
        let ctrl1 = Ctrl1::read(self)?;

        let val = XlMode::try_from(ctrl1.op_mode_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Set gyroscope output data rate (ODR).
    pub fn gy_data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut ctrl2 = Ctrl2::read(self)?;

        ctrl2.set_odr_g((val as u8) & 0x0F);
        ctrl2.write(self)?;

        let sel = ((val as u8) >> 4) & 0x0F;
        if sel != 0 {
            let mut haodr = HaodrCfg::read(self)?;
            haodr.set_haodr_sel(sel);
            haodr.write(self)?;
        }

        Ok(())
    }
    /// Get gyroscope output data rate (ODR).
    pub fn gy_data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let ctrl2 = Ctrl2::read(self)?;
        let haodr = HaodrCfg::read(self)?;
        let sel = haodr.haodr_sel();
        let odr = sel | ctrl2.odr_g();

        Ok(Odr::try_from(odr).unwrap_or_default())
    }
    /// Set gyroscope operating mode.
    pub fn gy_mode_set(&mut self, val: GyMode) -> Result<(), Error<B::Error>> {
        let mut ctrl2 = Ctrl2::read(self)?;
        ctrl2.set_op_mode_g(val as u8 & 0x07);
        ctrl2.write(self)
    }
    /// Get gyroscope operating mode.
    pub fn gy_mode_get(&mut self) -> Result<GyMode, Error<B::Error>> {
        let ctrl2 = Ctrl2::read(self)?;

        let val = GyMode::try_from(ctrl2.op_mode_g()).unwrap_or_default();
        Ok(val)
    }
    /// Enable/Disable the auto increment setting.
    ///
    /// If val == 1 it Enable automatic increment of the register address during
    /// multiple-byte access with a serial interface; enabled by default.
    pub fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self)?;
        ctrl3.set_if_inc(val);
        ctrl3.write(self)
    }
    /// Get the actual auto increment setting
    ///
    /// Register address automatically incremented during a multiple byte access
    /// with a serial interface (enable by default).
    pub fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl3::read(self).map(|reg| reg.if_inc())?;

        Ok(val)
    }
    /// Enable/Disable Block Data Update (BDU)
    ///
    /// If active the output registers are not updated until LSB and MSB have been read.
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self)?;
        ctrl3.set_bdu(val);
        ctrl3.write(self)
    }
    /// Get actual settings of Block Data Update (BDU)
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl3::read(self).map(|reg| reg.bdu())?;

        Ok(val)
    }
    /// Configure ODR trigger.
    ///
    /// Number of data generated in reference period when ODR-triggered mode is set.
    /// Allowed values: 0 (default) or 4 to 255.
    pub fn odr_trig_cfg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        if !(1..=3).contains(&val) {
            return Err(Error::UnexpectedValue);
        }

        let mut odr_trig = OdrTrigCfg::read(self)?;

        odr_trig.set_odr_trig_nodr(val);
        odr_trig.write(self)
    }
    /// Get the actual ODR trigger.
    ///
    /// Number of data generated in reference period when ODR-triggered mode is set.
    pub fn odr_trig_cfg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let odr_trig_nodr: u8 = OdrTrigCfg::read(self).map(|reg| reg.odr_trig_nodr())?;

        Ok(odr_trig_nodr)
    }
    /// Switch between pulsed and latched mode.
    ///
    /// Pulsed data-ready mode with ~75 us.
    pub fn data_ready_mode_set(&mut self, val: DataReadyMode) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = Ctrl4::read(self)?;
        ctrl4.set_drdy_pulsed((val as u8) & 0x1);
        ctrl4.write(self)
    }
    /// Get actual pulsed data-ready mode
    pub fn data_ready_mode_get(&mut self) -> Result<DataReadyMode, Error<B::Error>> {
        let ctrl4 = Ctrl4::read(self)?;
        let val = DataReadyMode::try_from(ctrl4.drdy_pulsed()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/disable interrupt and switch between latched/pulsed interrupts
    pub fn interrupt_enable_set(&mut self, val: InterruptMode) -> Result<(), Error<B::Error>> {
        let mut func = FunctionsEnable::read(self)?;
        func.set_interrupts_enable(val.enable);
        func.write(self)?;

        let mut cfg = TapCfg0::read(self)?;
        cfg.set_lir(val.lir);
        cfg.write(self)
    }
    /// Get the interrupt Mode
    ///
    /// Enable/disabled and latched/pulsed information.
    pub fn interrupt_enable_get(&mut self) -> Result<InterruptMode, Error<B::Error>> {
        let func = FunctionsEnable::read(self)?;
        let cfg = TapCfg0::read(self)?;

        let enable = func.interrupts_enable();
        let lir = cfg.lir();

        Ok(InterruptMode { enable, lir })
    }
    /// Set the Gyroscope full-scale.
    pub fn gy_full_scale_set(&mut self, val: GyFullScale) -> Result<(), Error<B::Error>> {
        let mut ctrl6 = Ctrl6::read(self)?;

        ctrl6.set_fs_g(val as u8 & 0xf);
        ctrl6.write(self)
    }
    /// Get the actual Gyroscope full-scale.
    pub fn gy_full_scale_get(&mut self) -> Result<GyFullScale, Error<B::Error>> {
        let ctrl6 = Ctrl6::read(self)?;

        let val = GyFullScale::try_from(ctrl6.fs_g()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Accelerometer full-scale.
    pub fn xl_full_scale_set(&mut self, val: XlFullScale) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self)?;
        ctrl8.set_fs_xl(val as u8 & 0x3);
        ctrl8.write(self)
    }
    /// Get the Accelerometer full-scale.
    pub fn xl_full_scale_get(&mut self) -> Result<XlFullScale, Error<B::Error>> {
        let ctrl8 = Ctrl8::read(self)?;

        let val = XlFullScale::try_from(ctrl8.fs_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables the accelerometer Dual channel mode:
    ///
    /// data with selected full scale and data with maximum full scale are sent
    /// simultaneously to two different set of output registers.
    pub fn xl_dual_channel_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self)?;
        ctrl8.set_xl_dualc_en(val);
        ctrl8.write(self)
    }
    /// Get the accelerometer Dual channel mode.
    ///
    /// Data with selected full scale and data with maximum full scale are sent
    /// simultaneously to two different set of output registers.
    pub fn xl_dual_channel_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl8::read(self).map(|reg| reg.xl_dualc_en())?;

        Ok(val)
    }
    /// Set the Accelerometer self-test.
    pub fn xl_self_test_set(&mut self, val: XlSelfTest) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self)?;
        ctrl10.set_st_xl(val as u8 & 0x3);
        ctrl10.write(self)
    }
    /// Get the actual Accelerometer self-test.
    pub fn xl_self_test_get(&mut self) -> Result<XlSelfTest, Error<B::Error>> {
        let ctrl10 = Ctrl10::read(self)?;

        let val = XlSelfTest::try_from(ctrl10.st_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Gyroscope self-test.
    pub fn gy_self_test_set(&mut self, val: GySelfTest) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self)?;
        ctrl10.set_st_g((val as u8) & 0x3);
        ctrl10.write(self)
    }
    /// Get the actual Gyroscope self-test selection.
    pub fn gy_self_test_get(&mut self) -> Result<GySelfTest, Error<B::Error>> {
        let ctrl10 = Ctrl10::read(self)?;

        let val = GySelfTest::try_from(ctrl10.st_g()).unwrap_or_default();

        Ok(val)
    }
    /// Set the SPI2 Accelerometer self-test.
    pub fn ois_xl_self_test_set(&mut self, val: OisXlSelfTest) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self)?;
        spi2_int_ois.set_st_xl_ois((val as u8) & 0x3);
        spi2_int_ois.write(self)
    }
    /// Get the actual SPI2 Accelerometer self-test.
    pub fn ois_xl_self_test_get(&mut self) -> Result<OisXlSelfTest, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self)?;
        let val = OisXlSelfTest::try_from(spi2_int_ois.st_xl_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Set SPI2 Accelerometer self-test.
    pub fn ois_gy_self_test_set(&mut self, val: OisGySelfTest) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self)?;

        spi2_int_ois.set_st_g_ois((val as u8) & 0x3);
        spi2_int_ois.set_st_ois_clampdis(((val as u8) & 0x04) >> 2);

        spi2_int_ois.write(self)
    }
    /// Get the actual SPI2 Accelerometer self-test.
    pub fn ois_gy_self_test_get(&mut self) -> Result<OisGySelfTest, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self)?;

        let val = match spi2_int_ois.st_g_ois() {
            0 => OisGySelfTest::Disable,
            1 => {
                if spi2_int_ois.st_ois_clampdis() == 1 {
                    OisGySelfTest::ClampPos
                } else {
                    OisGySelfTest::Positive
                }
            }
            2 => {
                if spi2_int_ois.st_ois_clampdis() == 1 {
                    OisGySelfTest::ClampNeg
                } else {
                    OisGySelfTest::Negative
                }
            }
            _ => OisGySelfTest::Disable,
        };

        Ok(val)
    }
    /// Set the signals that need to be routed on int1 pad.
    ///
    /// See `PinIntRoute` for a complete list of available events.
    /// Data ready temperature is available only on int2
    pub fn pin_int1_route_set(&mut self, val: &PinIntRoute) -> Result<(), Error<B::Error>> {
        // Check if drdy_temp is set to 1, which is not available on INT1
        if val.drdy_temp == 1 {
            return Err(Error::UnexpectedValue);
        }

        // Read INT1_CTRL register
        let mut int1_ctrl = Int1Ctrl::read(self)?;

        // Set fields from `val` to `int1_ctrl`
        int1_ctrl.set_int1_drdy_xl(val.drdy_xl);
        int1_ctrl.set_int1_drdy_g(val.drdy_g);
        int1_ctrl.set_int1_fifo_th(val.fifo_th);
        int1_ctrl.set_int1_fifo_ovr(val.fifo_ovr);
        int1_ctrl.set_int1_fifo_full(val.fifo_full);
        int1_ctrl.set_int1_cnt_bdr(val.cnt_bdr);

        // Write back to INT1_CTRL register
        int1_ctrl.write(self)?;

        // Read MD1_CFG register
        let mut md1_cfg = Md1Cfg::read(self)?;

        // Set fields from `val` to `md1_cfg`
        md1_cfg.set_int1_shub(val.shub);
        md1_cfg.set_int1_emb_func(val.emb_func);
        md1_cfg.set_int1_6d(val.sixd);
        md1_cfg.set_int1_single_tap(val.single_tap);
        md1_cfg.set_int1_double_tap(val.double_tap);
        md1_cfg.set_int1_wu(val.wakeup);
        md1_cfg.set_int1_ff(val.freefall);
        md1_cfg.set_int1_sleep_change(val.sleep_change);

        // Write back to MD1_CFG register
        md1_cfg.write(self)
    }
    /// Report the signals that are routed on int1 pad.
    /// Values returned are:
    ///     - drdy_xl
    ///     - drdy_g
    ///     - fifo_th
    ///     - fifo_ovr
    ///     - fifo_full
    ///     - cnt_bdr
    ///     - shub
    ///     - emb_func
    ///     - 6d
    ///     - single_tap
    ///     - double_tap
    ///     - wakeup
    ///     - free fall
    ///     - sleep change
    ///
    /// The remaining are leaved default
    pub fn pin_int1_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        // Reading INT1 Control Register
        let int1_ctrl = Int1Ctrl::read(self)?;

        // Reading MD1 Configuration Register
        let md1_cfg = Md1Cfg::read(self)?;

        let val = PinIntRoute {
            drdy_xl: int1_ctrl.int1_drdy_xl(),
            drdy_g: int1_ctrl.int1_drdy_g(),
            drdy_g_eis: 0,
            drdy_temp: 0,
            fifo_th: int1_ctrl.int1_fifo_th(),
            fifo_ovr: int1_ctrl.int1_fifo_ovr(),
            fifo_full: int1_ctrl.int1_fifo_full(),
            cnt_bdr: int1_ctrl.int1_cnt_bdr(),
            emb_func_endop: 0,
            timestamp: 0,
            shub: md1_cfg.int1_shub(),
            emb_func: md1_cfg.int1_emb_func(),
            sixd: md1_cfg.int1_6d(),
            single_tap: md1_cfg.int1_single_tap(),
            double_tap: md1_cfg.int1_double_tap(),
            wakeup: md1_cfg.int1_wu(),
            freefall: md1_cfg.int1_ff(),
            sleep_change: md1_cfg.int1_sleep_change(),
            drdy_ah_qvar: 0,
        };

        Ok(val)
    }
    /// Set the signals that need to be routed on int2 pad.
    pub fn pin_int2_route_set(&mut self, val: &PinIntRoute) -> Result<(), Error<B::Error>> {
        // Reading INT2 Control Register
        let mut int2_ctrl = Int2Ctrl::read(self)?;

        int2_ctrl.set_int2_drdy_xl(val.drdy_xl);
        int2_ctrl.set_int2_drdy_g(val.drdy_g);
        int2_ctrl.set_int2_fifo_th(val.fifo_th);
        int2_ctrl.set_int2_fifo_ovr(val.fifo_ovr);
        int2_ctrl.set_int2_fifo_full(val.fifo_full);
        int2_ctrl.set_int2_cnt_bdr(val.cnt_bdr);
        int2_ctrl.set_int2_drdy_g_eis(val.drdy_g_eis);
        int2_ctrl.set_int2_emb_func_endop(val.emb_func_endop);

        int2_ctrl.write(self)?;

        // Reading CTRL4 Register
        let mut ctrl4 = Ctrl4::read(self)?;

        ctrl4.set_int2_drdy_temp(val.drdy_temp);
        ctrl4.write(self)?;

        // Reading CTRL7 Register
        let mut ctrl7 = Ctrl7::read(self)?;

        ctrl7.set_int2_drdy_ah_qvar(val.drdy_ah_qvar);
        ctrl7.write(self)?;
        // Reading MD2 Configuration Register
        let mut md2_cfg = Md2Cfg::read(self)?;

        md2_cfg.set_int2_timestamp(val.timestamp);
        md2_cfg.set_int2_emb_func(val.emb_func);
        md2_cfg.set_int2_6d(val.sixd);
        md2_cfg.set_int2_single_tap(val.single_tap);
        md2_cfg.set_int2_double_tap(val.double_tap);
        md2_cfg.set_int2_wu(val.wakeup);
        md2_cfg.set_int2_ff(val.freefall);
        md2_cfg.set_int2_sleep_change(val.sleep_change);

        md2_cfg.write(self)
    }
    /// Report the signals that are routed on int2 pad.
    /// Values NOT returned are:
    ///     - shub
    pub fn pin_int2_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        // Read from INT2_CTRL register
        let int2_ctrl = Int2Ctrl::read(self)?;
        // Read from CTRL4 register
        let ctrl4 = Ctrl4::read(self)?;
        // Read from CTRL7 register
        let ctrl7 = Ctrl7::read(self)?;
        // Read from MD2_CFG register
        let md2_cfg = Md2Cfg::read(self)?;

        let val = PinIntRoute {
            drdy_xl: int2_ctrl.int2_drdy_xl(),
            drdy_g: int2_ctrl.int2_drdy_g(),
            fifo_th: int2_ctrl.int2_fifo_th(),
            fifo_ovr: int2_ctrl.int2_fifo_ovr(),
            fifo_full: int2_ctrl.int2_fifo_full(),
            cnt_bdr: int2_ctrl.int2_cnt_bdr(),
            drdy_g_eis: int2_ctrl.int2_drdy_g_eis(),
            emb_func_endop: int2_ctrl.int2_emb_func_endop(),
            timestamp: md2_cfg.int2_timestamp(),
            emb_func: md2_cfg.int2_emb_func(),
            sixd: md2_cfg.int2_6d(),
            single_tap: md2_cfg.int2_single_tap(),
            double_tap: md2_cfg.int2_double_tap(),
            wakeup: md2_cfg.int2_wu(),
            freefall: md2_cfg.int2_ff(),
            sleep_change: md2_cfg.int2_sleep_change(),
            drdy_ah_qvar: ctrl7.int2_drdy_ah_qvar(),
            drdy_temp: ctrl4.int2_drdy_temp(),
            shub: 0,
        };

        Ok(val)
    }

    /// Get routed embedded func interrupt signals on INT 1 pin.
    pub fn emb_pin_int1_route_get(&mut self) -> Result<EmbPinIntRoute, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_func_int1 = EmbFuncInt1::read(state)?;

            let val = EmbPinIntRoute {
                tilt: emb_func_int1.int1_tilt(),
                sig_mot: emb_func_int1.int1_sig_mot(),
                step_det: emb_func_int1.int1_step_detector(),
                fsm_lc: emb_func_int1.int1_fsm_lc(),
            };

            Ok(val)
        })
    }

    /// Routes embedded func interrupt signals on INT 1 pin.
    pub fn emb_pin_int1_route_set(&mut self, val: EmbPinIntRoute) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_int1 = EmbFuncInt1::read(state)?;

            emb_func_int1.set_int1_tilt(val.tilt);
            emb_func_int1.set_int1_sig_mot(val.sig_mot);
            emb_func_int1.set_int1_step_detector(val.step_det);
            emb_func_int1.set_int1_fsm_lc(val.fsm_lc);

            emb_func_int1.write(state)
        })?;

        let mut md1_cfg = Md1Cfg::read(self)?;
        md1_cfg.set_int1_emb_func(1);
        md1_cfg.write(self)
    }

    /// Get routed embedded func interrupt signals on INT 2 pin.
    pub fn emb_pin_int2_route_get(&mut self) -> Result<EmbPinIntRoute, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_func_int2 = EmbFuncInt2::read(state)?;

            let val = EmbPinIntRoute {
                tilt: emb_func_int2.int2_tilt(),
                sig_mot: emb_func_int2.int2_sig_mot(),
                step_det: emb_func_int2.int2_step_detector(),
                fsm_lc: emb_func_int2.int2_fsm_lc(),
            };

            Ok(val)
        })
    }

    /// Routes embedded func interrupt signals on INT 2 pin.
    pub fn emb_pin_int2_route_set(&mut self, val: EmbPinIntRoute) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_int2 = EmbFuncInt2::read(state)?;

            emb_func_int2.set_int2_tilt(val.tilt);
            emb_func_int2.set_int2_sig_mot(val.sig_mot);
            emb_func_int2.set_int2_step_detector(val.step_det);
            emb_func_int2.set_int2_fsm_lc(val.fsm_lc);

            emb_func_int2.write(state)
        })?;

        let mut md2_cfg = Md2Cfg::read(self)?;
        md2_cfg.set_int2_emb_func(1);
        md2_cfg.write(self)
    }

    /// Get interrupt configuration mode.
    pub fn embedded_int_cfg_get(&mut self) -> Result<EmbeddedIntConf, Error<B::Error>> {
        let mut val = EmbeddedIntConf::Pulsed;

        MemBank::operate_over_embed(self, |state| {
            let page_rw = PageRw::read(state)?;
            if page_rw.emb_func_lir() == 1 {
                val = EmbeddedIntConf::Latched;
            }

            Ok(val)
        })
    }

    /// Set interrupt configuration mode.
    pub fn embedded_int_cfg_set(&mut self, val: EmbeddedIntConf) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut page_rw = PageRw::read(state)?;
            match val {
                EmbeddedIntConf::Latched => {
                    page_rw.set_emb_func_lir(1);
                }
                EmbeddedIntConf::Pulsed => {
                    page_rw.set_emb_func_lir(0);
                }
            };
            page_rw.write(state)
        })
    }

    /// Get the status of embedded functions.
    pub fn embedded_status_get(&mut self) -> Result<EmbeddedStatus, Error<B::Error>> {
        let status = EmbFuncStatusMainPage::read(self)?;
        MemBank::operate_over_embed(self, |state| {
            let src = EmbFuncSrc::read(state)?;

            let val = EmbeddedStatus {
                tilt: status.is_tilt(),
                sig_mot: status.is_sigmot(),
                fsm_lc: status.is_fsm_lc(),
                step_count_inc: src.stepcounter_bit_set(),
                step_count_overflow: src.step_overflow(),
                step_on_delta_time: src.step_count_delta_ia(),
                step_detector: src.step_detected(),
            };

            Ok(val)
        })
    }

    /// Get the status of all the interrupt sources.
    pub fn all_sources_get(&mut self) -> Result<AllSources, Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self)?;
        functions_enable.set_dis_rst_lir_all_int(1);
        functions_enable.write(self)?;

        let fifo_status = FifoStatusReg::read(self)?;
        let all_int_src = AllIntSrc::read(self)?;
        let status_reg = StatusReg::read(self)?;

        functions_enable = FunctionsEnable::read(self)?;
        functions_enable.set_dis_rst_lir_all_int(0);
        functions_enable.write(self)?;

        let status_reg_ois = UiStatusRegOis::read(self)?;
        let wake_up_src = WakeUpSrc::read(self)?;
        let tap_src = TapSrc::read(self)?;
        let d6d_src = D6dSrc::read(self)?;

        let emb_func_status_mainpage = EmbFuncStatusMainPage::read(self)?;
        let fsm_status_mainpage = FsmStatusMainPage::read(self)?;
        let mlc_status_mainpage = MlcStatusMainPage::read(self)?;

        // Embedded function
        let (emb_func_exec_status, emb_func_src) = MemBank::operate_over_embed(self, |state| {
            let emb_func_exec_status = EmbFuncExecStatus::read(state)?;
            let emb_func_src = EmbFuncSrc::read(state)?;
            Ok((emb_func_exec_status, emb_func_src))
        })?;

        let status_shub = StatusMasterMainPage::read(self)?;

        let val = AllSources {
            drdy_xl: status_reg.xlda(),
            drdy_gy: status_reg.gda(),
            drdy_temp: status_reg.tda(),
            drdy_ah_qvar: status_reg.ah_qvarda(),
            drdy_eis: status_reg.gda_eis(),
            drdy_ois: status_reg.ois_drdy(),
            gy_settling: status_reg_ois.gyro_settling(),
            timestamp: status_reg.timestamp_endcount(),
            free_fall: all_int_src.ff_ia(),
            wake_up: all_int_src.wu_ia(),
            wake_up_z: wake_up_src.z_wu(),
            wake_up_y: wake_up_src.y_wu(),
            wake_up_x: wake_up_src.x_wu(),
            single_tap: tap_src.single_tap(),
            double_tap: tap_src.double_tap(),
            tap_z: tap_src.z_tap(),
            tap_y: tap_src.y_tap(),
            tap_x: tap_src.x_tap(),
            tap_sign: tap_src.tap_sign(),
            six_d: all_int_src.d6d_ia(),
            six_d_xl: d6d_src.xl(),
            six_d_xh: d6d_src.xh(),
            six_d_yl: d6d_src.yl(),
            six_d_yh: d6d_src.yh(),
            six_d_zl: d6d_src.zl(),
            six_d_zh: d6d_src.zh(),
            sleep_change: wake_up_src.sleep_change_ia(),
            sleep_state: wake_up_src.sleep_state(),
            step_detector: emb_func_src.step_detected(),
            step_count_inc: emb_func_src.stepcounter_bit_set(),
            step_count_overflow: emb_func_src.step_overflow(),
            step_on_delta_time: emb_func_src.step_count_delta_ia(),
            emb_func_stand_by: emb_func_exec_status.emb_func_endop(),
            emb_func_time_exceed: emb_func_exec_status.emb_func_exec_ovr(),
            tilt: emb_func_status_mainpage.is_tilt(),
            sig_mot: emb_func_status_mainpage.is_sigmot(),
            fsm_lc: emb_func_status_mainpage.is_fsm_lc(),
            fsm1: fsm_status_mainpage.is_fsm1(),
            fsm2: fsm_status_mainpage.is_fsm2(),
            fsm3: fsm_status_mainpage.is_fsm3(),
            fsm4: fsm_status_mainpage.is_fsm4(),
            fsm5: fsm_status_mainpage.is_fsm5(),
            fsm6: fsm_status_mainpage.is_fsm6(),
            fsm7: fsm_status_mainpage.is_fsm7(),
            fsm8: fsm_status_mainpage.is_fsm8(),
            mlc1: mlc_status_mainpage.is_mlc1(),
            mlc2: mlc_status_mainpage.is_mlc2(),
            mlc3: mlc_status_mainpage.is_mlc3(),
            mlc4: mlc_status_mainpage.is_mlc4(),
            sh_endop: status_shub.sens_hub_endop(),
            sh_slave0_nack: status_shub.slave0_nack(),
            sh_slave1_nack: status_shub.slave1_nack(),
            sh_slave2_nack: status_shub.slave2_nack(),
            sh_slave3_nack: status_shub.slave3_nack(),
            sh_wr_once: status_shub.wr_once_done(),
            fifo_bdr: fifo_status.counter_bdr_ia(),
            fifo_full: fifo_status.fifo_full_ia(),
            fifo_ovr: fifo_status.fifo_ovr_ia(),
            fifo_th: fifo_status.fifo_wtm_ia(),
        };

        Ok(val)
    }
    /// Get Flag data ready
    ///
    /// Return data ready status about: xl, gy, temp
    pub fn flag_data_ready_get(&mut self) -> Result<DataReady, Error<B::Error>> {
        let status_reg = StatusReg::read(self)?;

        let data_ready = DataReady {
            drdy_xl: status_reg.xlda(),
            drdy_gy: status_reg.gda(),
            drdy_temp: status_reg.tda(),
        };

        Ok(data_ready)
    }
    /// Set Mask status bit reset
    pub fn int_ack_mask_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| IntAckMask::from_bits(val).write(state))
    }
    /// Get the actual Mask status bit reset
    pub fn int_ack_mask_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            IntAckMask::read(state).map(|reg| reg.into_bits())
        })
    }
    /// Get the Temperature raw data
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        OutTemp::read(self).map(|reg| reg.0)
    }

    /// Get the Angular rate raw data.
    pub fn angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZG::read(self)?;

        Ok([val.x, val.y, val.z])
    }
    /// Get the OIS Angular rate raw data.
    ///
    /// Data comes through SPI2
    pub fn ois_angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = Spi2OutXYZGOis::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Get angular rate raw data for OIS gyro or the EIS gyro channel.
    pub fn ois_eis_angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = UiOutXYZGOisEis::read(self)?;

        Ok([val.x, val.y, val.z])
    }

    /// Get the Linear acceleration raw data.
    pub fn acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZA::read(self)?;

        Ok([val.x, val.y, val.z])
    }
    /// Get the Linear acceleration sensor for Dual channel mode.
    pub fn dual_acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = UiOutXYZAOisDualc::read(self)?;

        Ok([val.x, val.y, val.z])
    }
    /// Get ah_qvar raw data.
    ///
    /// When the analog hub or Qvar is enabled, the output is
    /// the analog hub or Qvar sensor data.
    pub fn ah_qvar_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        AhQvarOut::read(self).map(|reg| reg.0)
    }
    /// Get difference in percentage of the effective ODR
    /// (and timestamp rate) with respect to the typical.
    ///
    /// Step: 0.13%. 8-bit format, 2's complement.
    pub fn odr_cal_reg_get(&mut self) -> Result<i8, Error<B::Error>> {
        let val: i8 = InternalFreq::read(self).map(|reg| reg.freq_fine() as i8)?;

        Ok(val)
    }

    /// Enable debug mode for embedded functions.
    pub fn emb_function_dbg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self)?;
        ctrl10.set_emb_func_debug(val);
        ctrl10.write(self)
    }
    /// Get if debug mode for embedded functions is enabled
    pub fn emb_function_dbg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl10::read(self).map(|reg| reg.emb_func_debug())?;

        Ok(val)
    }
    /// It changes the polarity of INT2 pin input trigger for data enable (DEN) or embedded functions.
    pub fn den_polarity_set(&mut self, val: DenPolarity) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = Ctrl4::read(self)?;
        ctrl4.set_int2_in_lh((val as u8) & 0x1);
        ctrl4.write(self)
    }
    /// Get the polarity of INT2 pin input trigger for data enable (DEN) or embedded functions.
    pub fn den_polarity_get(&mut self) -> Result<DenPolarity, Error<B::Error>> {
        let ctrl4 = Ctrl4::read(self)?;

        let val = DenPolarity::try_from(ctrl4.int2_in_lh()).unwrap_or_default();

        Ok(val)
    }
    /// Set Data ENable (DEN) configuration.
    pub fn den_conf_set(&mut self, val: DenConf) -> Result<(), Error<B::Error>> {
        let mut den = Den::read(self)?;

        den.set_den_z(val.den_z);
        den.set_den_y(val.den_y);
        den.set_den_x(val.den_x);

        den.set_lvl2_en((val.mode as u8) & 0x1);
        den.set_lvl1_en(((val.mode as u8) & 0x2) >> 1);

        if val.stamp_in_gy_data == 1 && val.stamp_in_xl_data == 1 {
            den.set_den_xl_g(0);
            den.set_den_xl_en(1);
        } else if val.stamp_in_gy_data == 1 && val.stamp_in_xl_data == 0 {
            den.set_den_xl_g(0);
            den.set_den_xl_en(0);
        } else if val.stamp_in_gy_data == 0 && val.stamp_in_xl_data == 1 {
            den.set_den_xl_g(1);
            den.set_den_xl_en(0);
        } else {
            den.set_den_xl_g(0);
            den.set_den_xl_en(0);
            den.set_den_z(0);
            den.set_den_y(0);
            den.set_den_x(0);
            den.set_lvl2_en(0);
            den.set_lvl1_en(0);
        }

        den.write(self)
    }
    /// Get Data ENable (DEN) configuration.
    pub fn den_conf_get(&mut self) -> Result<DenConf, Error<B::Error>> {
        let den = Den::read(self)?;

        let mut val = DenConf {
            den_z: den.den_z(),
            den_y: den.den_y(),
            den_x: den.den_x(),
            stamp_in_gy_data: 0,
            stamp_in_xl_data: 0,
            mode: DenMode::DenNotDefined,
        };

        if (den.den_x() | den.den_y() | den.den_z()) == 1 {
            if den.den_xl_g() == 0 && den.den_xl_en() == 1 {
                val.stamp_in_gy_data = 1;
                val.stamp_in_xl_data = 1;
            } else if den.den_xl_g() == 0 && den.den_xl_en() == 0 {
                val.stamp_in_gy_data = 1;
                val.stamp_in_xl_data = 0;
            } else {
                val.stamp_in_gy_data = 0;
                val.stamp_in_xl_data = 1;
            }
        }

        match (den.lvl1_en() << 1) + den.lvl2_en() {
            3 => val.mode = DenMode::LevelTrigger,
            2 => val.mode = DenMode::LevelLatched,
            _ => val.mode = DenMode::DenNotDefined,
        }

        Ok(val)
    }
    /// Gyroscope full-scale selection for EIS channel.
    ///
    /// WARNING: 4000dps will be available only if also User Interface chain is set to 4000dps.
    pub fn eis_gy_full_scale_set(&mut self, val: EisGyFullScale) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self)?;
        ctrl_eis.set_fs_g_eis(val as u8 & 0x7);
        ctrl_eis.write(self)
    }
    /// Get the actual Gyroscope full-scale selection for EIS channel.
    ///
    /// WARNING: 4000dps will be available only if also User Interface chain is set to 4000dps
    pub fn eis_gy_full_scale_get(&mut self) -> Result<EisGyFullScale, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self)?;
        let val = EisGyFullScale::try_from(ctrl_eis.fs_g_eis()).unwrap_or_default();

        Ok(val)
    }
    /// Enables routing of gyroscope EIS outputs on SPI2 (OIS interface).
    ///
    /// The gyroscope data on SPI2 (OIS interface) cannot be read from User Interface (UI).
    pub fn eis_gy_on_spi2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self)?;
        ctrl_eis.set_g_eis_on_g_ois_out_reg(val);
        ctrl_eis.write(self)
    }
    /// Enables routing of gyroscope EIS outputs on SPI2 (OIS interface).
    ///
    /// The gyroscope data on SPI2 (OIS interface) cannot be read from User Interface (UI).
    pub fn eis_gy_on_spi2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlEis::read(self).map(|reg| reg.g_eis_on_g_ois_out_reg())?;

        Ok(val)
    }
    /// Enables and selects the ODR of the gyroscope EIS channel.
    pub fn gy_eis_data_rate_set(&mut self, val: GyEisDataRate) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self)?;
        ctrl_eis.set_odr_g_eis((val as u8) & 0x03);
        ctrl_eis.write(self)
    }
    /// Get the actual ODR of the gyroscope EIS channel.
    pub fn gy_eis_data_rate_get(&mut self) -> Result<GyEisDataRate, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self)?;

        let val = GyEisDataRate::try_from(ctrl_eis.odr_g_eis()).unwrap_or_default();

        Ok(val)
    }
    /// Set FIFO watermark threshold
    ///
    /// 1 LSb = TAG (1 Byte) + 1 sensor (6 Bytes) written in FIFO.
    pub fn fifo_watermark_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl1 = FifoCtrl1::read(self)?;
        fifo_ctrl1.set_wtm(val);
        fifo_ctrl1.write(self)
    }
    /// Get the actual FIFO watermark threshold
    ///
    /// 1 LSb = TAG (1 Byte) + 1 sensor (6 Bytes) written in FIFO
    pub fn fifo_watermark_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl1::read(self).map(|reg| reg.wtm())?;

        Ok(val)
    }
    /// Enables FSM-triggered batching in FIFO of accelerometer channel 2, WHEN dual channel mode
    /// is enabled
    pub fn fifo_xl_dual_fsm_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_xl_dualc_batch_from_fsm(val);
        fifo_ctrl2.write(self)
    }
    /// Get if FSM-triggered batching in FIFO of accelerometer channel 2 is enabled. Required dual channel mode enabled.
    pub fn fifo_xl_dual_fsm_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).map(|reg| reg.xl_dualc_batch_from_fsm())?;

        Ok(val)
    }
    /// It configures the compression algorithm to write non-compressed data at each rate.
    pub fn fifo_compress_algo_set(&mut self, val: FifoCompressAlgo) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_uncompr_rate((val as u8) & 0x03);
        fifo_ctrl2.write(self)
    }
    /// Get the actual compression algorithm.
    ///
    /// The compression algorithm handles the write of non-compressed data at each rate.
    pub fn fifo_compress_algo_get(&mut self) -> Result<FifoCompressAlgo, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self)?;

        let val = FifoCompressAlgo::try_from(fifo_ctrl2.uncompr_rate()).unwrap_or_default();

        Ok(val)
    }
    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    pub fn fifo_virtual_sens_odr_chg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_odr_chg_en(val);
        fifo_ctrl2.write(self)
    }
    /// Get the configuration (enable/disable) of ODR CHANGE virtual sensor to be batched in FIFO.
    pub fn fifo_virtual_sens_odr_chg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).map(|reg| reg.odr_chg_en())?;

        Ok(val)
    }
    /// Enables/Disables compression algorithm runtime.
    ///
    /// If val is 1: compression algorithm is active at runtime
    pub fn fifo_compress_algo_real_time_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_fifo_compr_rt_en(val);
        fifo_ctrl2.write(self)?;

        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state)?;
            emb_func_en_b.set_fifo_compr_en(val);
            emb_func_en_b.write(state)
        })
    }
    /// Get the configuration (enable/disable) compression algorithm runtime.
    pub fn fifo_compress_algo_real_time_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).map(|reg| reg.fifo_compr_rt_en())?;

        Ok(val)
    }
    /// Sensing chain FIFO stop values memorization at threshold level.
    pub fn fifo_stop_on_wtm_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self)?;
        fifo_ctrl2.set_stop_on_wtm(val);
        fifo_ctrl2.write(self)
    }
    /// Get the configuration (enable/disable) for sensing chain FIFO stop values memorization at threshold level.
    pub fn fifo_stop_on_wtm_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).map(|reg| reg.stop_on_wtm())?;

        Ok(val)
    }
    /// Selects Batch Data Rate (write frequency in FIFO) for accelerometer data.
    pub fn fifo_xl_batch_set(&mut self, val: FifoBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self)?;
        fifo_ctrl3.set_bdr_xl((val as u8) & 0xF);
        fifo_ctrl3.write(self)
    }
    /// Get the Batch Data Rate (write frequency in FIFO) for accelerometer data.
    pub fn fifo_xl_batch_get(&mut self) -> Result<FifoBatch, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self)?;
        let val = FifoBatch::try_from(fifo_ctrl3.bdr_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Selects Batch Data Rate (write frequency in FIFO) for gyroscope data.
    pub fn fifo_gy_batch_set(&mut self, val: FifoBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self)?;
        fifo_ctrl3.set_bdr_gy((val as u8) & 0x0F);
        fifo_ctrl3.write(self)
    }
    /// Get Batch Data Rate (write frequency in FIFO) for gyroscope data.
    pub fn fifo_gy_batch_get(&mut self) -> Result<FifoBatch, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self)?;

        let val = FifoBatch::try_from(fifo_ctrl3.bdr_gy()).unwrap_or_default();

        Ok(val)
    }
    /// Set FIFO mode.
    pub fn fifo_mode_set(&mut self, val: FifoMode) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_fifo_mode((val as u8) & 0x07);
        fifo_ctrl4.write(self)
    }
    /// Get FIFO mode.
    pub fn fifo_mode_get(&mut self) -> Result<FifoMode, Error<B::Error>> {
        let fifo_ctrl = FifoCtrl4::read(self)?;
        let fifo_mode = FifoMode::try_from(fifo_ctrl.fifo_mode()).unwrap_or_default();

        Ok(fifo_mode)
    }
    /// Enables/Disables FIFO batching of EIS gyroscope output values.
    ///
    /// If val is 1 FIFO batching is enabled
    pub fn fifo_gy_eis_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_g_eis_fifo_en(val);
        fifo_ctrl4.write(self)
    }
    /// Get the configuration (enabled/disabled) for FIFO batching of EIS gyroscope output values.
    pub fn fifo_gy_eis_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl4::read(self).map(|reg| reg.g_eis_fifo_en())?;

        Ok(val)
    }
    /// Set batch data rate (write frequency in FIFO) for temperature data.
    pub fn fifo_temp_batch_set(&mut self, val: FifoTempBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_odr_t_batch((val as u8) & 0x03);
        fifo_ctrl4.write(self)
    }
    /// Get actual batch data rate (write frequency in FIFO) for temperature data.
    pub fn fifo_temp_batch_get(&mut self) -> Result<FifoTempBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self)?;

        let val = FifoTempBatch::try_from(fifo_ctrl4.odr_t_batch()).unwrap_or_default();

        Ok(val)
    }
    /// Selects decimation for timestamp batching in FIFO.
    ///
    /// Write rate will be the maximum rate between XL and GYRO BDR divided by decimation decoder.
    pub fn fifo_timestamp_batch_set(
        &mut self,
        val: FifoTimestampBatch,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self)?;
        fifo_ctrl4.set_dec_ts_batch((val as u8) & 0x03);
        fifo_ctrl4.write(self)
    }
    /// Get the actual decimation for timestamp batching in FIFO.
    ///
    /// Write rate will be the maximum rate between XL and GYRO BDR divided by decimation decoder.
    pub fn fifo_timestamp_batch_get(&mut self) -> Result<FifoTimestampBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self)?;

        let val = FifoTimestampBatch::try_from(fifo_ctrl4.dec_ts_batch()).unwrap_or_default();

        Ok(val)
    }
    /// Set the threshold for the internal counter of batch events.
    ///
    /// When this counter reaches the threshold, the counter is
    /// reset and the interrupt flag is set to 1.
    pub fn fifo_batch_counter_threshold_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let mut counter_bdr = CounterBdrReg::read(self)?;
        counter_bdr.set_cnt_bdr_th(val);
        counter_bdr.write(self)
    }
    /// Get the threshold for the internal counter of batch events.
    ///
    /// When this counter reaches the threshold, the counter is
    /// reset and the interrupt flag is set to 1.
    pub fn fifo_batch_counter_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        let counter = CounterBdrReg::read(self)?;

        Ok(counter.cnt_bdr_th())
    }
    /// Set the trigger for the internal counter of batch events.
    pub fn fifo_batch_cnt_event_set(
        &mut self,
        val: FifoBatchCntEvent,
    ) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self)?;
        counter_bdr_reg1.set_trig_counter_bdr(val as u8 & 0x03);
        counter_bdr_reg1.write(self)
    }
    /// Get the actual trigger for the internal counter of batch events.
    pub fn fifo_batch_cnt_event_get(&mut self) -> Result<FifoBatchCntEvent, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self)?;

        let val =
            FifoBatchCntEvent::try_from(counter_bdr_reg1.trig_counter_bdr()).unwrap_or_default();

        Ok(val)
    }
    /// Retrieves FIFO status.
    pub fn fifo_status_get(&mut self) -> Result<FifoStatus, Error<B::Error>> {
        let status = FifoStatusReg::read(self)?;

        let fifo_status = FifoStatus {
            fifo_bdr: status.counter_bdr_ia(),
            fifo_ovr: status.fifo_ovr_ia(),
            fifo_full: status.fifo_full_ia(),
            fifo_th: status.fifo_wtm_ia(),
            fifo_level: status.diff_fifo(),
        };

        Ok(fifo_status)
    }
    /// Get the FIFO data tag and data.
    pub fn fifo_out_raw_get(&mut self) -> Result<FifoOutRaw, Error<B::Error>> {
        let fifo_data_out_tag = FifoDataOutTag::read(self)?;
        let tag_sensor = Tag::try_from(fifo_data_out_tag.tag_sensor()).unwrap_or_default();
        let cnt = fifo_data_out_tag.tag_cnt();
        let data = FifoDataOutXYZ::read(self)?;

        Ok(FifoOutRaw {
            tag: tag_sensor,
            cnt,
            data: data.0,
        })
    }
    /// Set the batching in FIFO buffer of step counter value.
    pub fn fifo_stpcnt_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state)?;
            emb_func_fifo_en_a.set_step_counter_fifo_en(val);
            emb_func_fifo_en_a.write(state)
        })
    }
    /// Get the acutal batching in FIFO buffer of step counter value.
    pub fn fifo_stpcnt_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            EmbFuncFifoEnA::read(state).map(|reg| reg.step_counter_fifo_en())
        })
    }
    /// Enables/Disables batching in FIFO buffer of machine learning core results.
    pub fn fifo_mlc_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state)?;
            emb_func_fifo_en_a.set_mlc_fifo_en(val);
            emb_func_fifo_en_a.write(state)
        })
    }
    /// Get the configuration (enables/disables) batching in FIFO buffer of machine learning core results.
    pub fn fifo_mlc_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_func_fifo_en_a = EmbFuncFifoEnA::read(state)?;
            Ok(emb_func_fifo_en_a.mlc_fifo_en())
        })
    }
    /// Enables batching in FIFO buffer of machine learning core filters and features.
    pub fn fifo_mlc_filt_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_fifo_en_b = EmbFuncFifoEnB::read(state)?;
            emb_func_fifo_en_b.set_mlc_filter_feature_fifo_en(val);
            emb_func_fifo_en_b.write(state)
        })
    }
    /// Get the configuration (enable/disable) of batching in FIFO buffer
    /// of machine learning core filters and features.
    pub fn fifo_mlc_filt_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            EmbFuncFifoEnB::read(state).map(|reg| reg.mlc_filter_feature_fifo_en())
        })
    }
    /// Enable FIFO data batching of target idx.
    pub fn fifo_sh_batch_slave_set(&mut self, idx: u8, val: u8) -> Result<(), Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank)?;

        let mut buf = [0u8; 1];
        self.read_from_register(SnsHubReg::Slv0Config as u8 + idx * 3, &mut buf)?;
        let mut slv_config = Slv0Config::from_bits(buf[0]);
        slv_config.set_batch_ext_sens_0_en(val);
        self.write_to_register(SnsHubReg::Slv0Config as u8 + idx * 3, &[slv_config.into()])?;

        self.mem_bank_set(MemBank::MainMemBank)
    }
    /// Get the actual configuration (enable/disable) FIFO data batching of target idx.
    pub fn fifo_sh_batch_slave_get(&mut self, idx: u8) -> Result<u8, Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank)?;

        let mut buf = [0u8; 1];
        self.read_from_register(SnsHubReg::Slv0Config as u8 + idx * 3, &mut buf)?;
        let val: u8 = Slv0Config::from_bits(buf[0]).batch_ext_sens_0_en();

        self.mem_bank_set(MemBank::MainMemBank)?;

        Ok(val)
    }
    /// Enable/Disable Batching in FIFO buffer of SFLP.
    pub fn fifo_sflp_batch_set(&mut self, val: FifoSflpRaw) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state)?;
            emb_func_fifo_en_a.set_sflp_game_fifo_en(val.game_rotation);
            emb_func_fifo_en_a.set_sflp_gravity_fifo_en(val.gravity);
            emb_func_fifo_en_a.set_sflp_gbias_fifo_en(val.gbias);
            emb_func_fifo_en_a.write(state)
        })
    }
    /// Get the actual configuration (enable/disable) for Batching in FIFO buffer of SFLP.
    pub fn fifo_sflp_batch_get(&mut self) -> Result<FifoSflpRaw, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_func_fifo_en_a = EmbFuncFifoEnA::read(state)?;

            let val = FifoSflpRaw {
                game_rotation: emb_func_fifo_en_a.sflp_game_fifo_en(),
                gravity: emb_func_fifo_en_a.sflp_gravity_fifo_en(),
                gbias: emb_func_fifo_en_a.sflp_gbias_fifo_en(),
            };

            Ok(val)
        })
    }
    /// Set Protocol anti-spike filters.
    pub fn filt_anti_spike_set(&mut self, val: FiltAntiSpike) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self)?;
        if_cfg.set_asf_ctrl((val as u8) & 0x01);
        if_cfg.write(self)
    }
    /// Get the actual Protocol anti-spike filters.
    pub fn filt_anti_spike_get(&mut self) -> Result<FiltAntiSpike, Error<B::Error>> {
        let if_cfg = IfCfg::read(self)?;
        let val = FiltAntiSpike::try_from(if_cfg.asf_ctrl()).unwrap_or_default();

        Ok(val)
    }
    /// It masks DRDY and Interrupts RQ until filter settling ends.
    pub fn filt_settling_mask_set(&mut self, val: FiltSettlingMask) -> Result<(), Error<B::Error>> {
        // Read CTRL4 register and set the drdy_mask field
        let mut ctrl4 = Ctrl4::read(self)?;
        ctrl4.set_drdy_mask(val.drdy);
        ctrl4.write(self)?;

        // Read EMB_FUNC_CFG register and set the irq_mask fields
        let mut emb_func_cfg = EmbFuncCfg::read(self)?;
        emb_func_cfg.set_emb_func_irq_mask_xl_settl(val.irq_xl);
        emb_func_cfg.set_emb_func_irq_mask_g_settl(val.irq_g);
        emb_func_cfg.write(self)?;

        // Read UI_INT_OIS register and set the drdy_mask_ois field
        let mut ui_int_ois = UiIntOis::read(self)?;
        ui_int_ois.set_drdy_mask_ois(val.ois_drdy);
        ui_int_ois.write(self)
    }
    /// Get the configuration for masks DRDY and Interrupts RQ.
    pub fn filt_settling_mask_get(&mut self) -> Result<FiltSettlingMask, Error<B::Error>> {
        let emb_func_cfg = EmbFuncCfg::read(self)?;
        let ui_int_ois = UiIntOis::read(self)?;
        let ctrl4 = Ctrl4::read(self)?;

        let val: FiltSettlingMask = FiltSettlingMask {
            irq_xl: emb_func_cfg.emb_func_irq_mask_xl_settl(),
            irq_g: emb_func_cfg.emb_func_irq_mask_g_settl(),
            drdy: ctrl4.drdy_mask(),
            ois_drdy: ui_int_ois.drdy_mask_ois(),
        };

        Ok(val)
    }
    /// Set the masks for DRDY and Interrupts RQ until filter settling ends.
    pub fn filt_ois_settling_mask_set(&mut self, ois_drdy: u8) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self)?;
        spi2_int_ois.set_drdy_mask_ois(ois_drdy);
        spi2_int_ois.write(self)
    }
    /// Get the masks DRDY and Interrupts RQ until filter settling ends.
    pub fn filt_ois_settling_mask_get(&mut self) -> Result<FiltOisSettlingMask, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self)?;
        let val = FiltOisSettlingMask {
            ois_drdy: spi2_int_ois.drdy_mask_ois(),
        };
        Ok(val)
    }
    /// Set the Gyroscope low-pass filter (LPF1) bandwidth
    pub fn filt_gy_lp1_bandwidth_set(
        &mut self,
        val: FiltGyLp1Bandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl6 = Ctrl6::read(self)?;
        ctrl6.set_lpf1_g_bw((val as u8) & 0x0F);
        ctrl6.write(self)
    }
    /// Get the gyroscope low-pass filter (LPF1) bandwidth.
    pub fn filt_gy_lp1_bandwidth_get(&mut self) -> Result<FiltGyLp1Bandwidth, Error<B::Error>> {
        let ctrl6 = Ctrl6::read(self)?;
        let bandwidth = FiltGyLp1Bandwidth::try_from(ctrl6.lpf1_g_bw()).unwrap_or_default();

        Ok(bandwidth)
    }
    /// Enables/Disable gyroscope digital LPF1 filter.
    pub fn filt_gy_lp1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self)?;
        ctrl7.set_lpf1_g_en(val);
        ctrl7.write(self)
    }
    /// Get the configuration (enables/disables) gyroscope digital LPF1 filter.
    pub fn filt_gy_lp1_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ctrl7::read(self).map(|reg| reg.lpf1_g_en())
    }
    /// Set the Accelerometer LPF2 and high pass filter configuration and cutoff setting.
    pub fn filt_xl_lp2_bandwidth_set(
        &mut self,
        val: FiltXlLp2Bandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self)?;
        ctrl8.set_hp_lpf2_xl_bw((val as u8) & 0x07);
        ctrl8.write(self)
    }
    /// Get the current Accelerometer LPF2 and high pass filter configuration and cutoff setting.
    pub fn filt_xl_lp2_bandwidth_get(&mut self) -> Result<FiltXlLp2Bandwidth, Error<B::Error>> {
        let ctrl8 = Ctrl8::read(self)?;
        let bandwidth =
            FiltXlLp2Bandwidth::try_from(ctrl8.hp_lpf2_xl_bw() & 0x07).unwrap_or_default();

        Ok(bandwidth)
    }
    /// Enable/Disable accelerometer LPS2 (Low Pass Filter 2) filtering stage.
    pub fn filt_xl_lp2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self)?;
        ctrl9.set_lpf2_xl_en(val);
        ctrl9.write(self)
    }
    /// Get the accelerometer LPS2 (Low Pass Filter 2) filtering stage.
    pub fn filt_xl_lp2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).map(|reg| reg.lpf2_xl_en())?;

        Ok(val)
    }
    /// Accelerometer slope filter / high-pass filter selection.
    pub fn filt_xl_hp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self)?;
        ctrl9.set_hp_slope_xl_en(val);
        ctrl9.write(self)
    }
    /// Get the Accelerometer slope filter / high-pass filter selection.
    pub fn filt_xl_hp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).map(|reg| reg.hp_slope_xl_en())?;

        Ok(val)
    }
    /// Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the first sample.
    pub fn filt_xl_fast_settling_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self)?;
        ctrl9.set_xl_fastsettl_mode(val);
        ctrl9.write(self)
    }
    /// Get accelerometer LPF2 and HPF fast-settling mode. The filter sets the first sample.
    pub fn filt_xl_fast_settling_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).map(|reg| reg.xl_fastsettl_mode())?;

        Ok(val)
    }
    /// Set Accelerometer high-pass filter mode.
    pub fn filt_xl_hp_mode_set(&mut self, val: FiltXlHpMode) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self)?;
        ctrl9.set_hp_ref_mode_xl(val as u8 & 0x01);
        ctrl9.write(self)
    }
    /// Get Accelerometer high-pass filter mode.
    pub fn filt_xl_hp_mode_get(&mut self) -> Result<FiltXlHpMode, Error<B::Error>> {
        let ctrl9 = Ctrl9::read(self)?;

        let mode = FiltXlHpMode::try_from(ctrl9.hp_ref_mode_xl()).unwrap_or_default();

        Ok(mode)
    }
    /// Filter wakeup activity feed set.
    ///
    /// Set HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions.
    pub fn filt_wkup_act_feed_set(&mut self, val: FiltWkupActFeed) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self)?;
        let mut tap_cfg0 = TapCfg0::read(self)?;

        tap_cfg0.set_slope_fds((val as u8) & 0x01);
        tap_cfg0.write(self)?;

        wake_up_ths.set_usr_off_on_wu(((val as u8) & 0x02) >> 1);
        wake_up_ths.write(self)
    }
    /// Filter wakeup activity feed get.
    ///
    /// Get the actual HPF or SLOPE filter on wake-up and Activity/Inactivity functions.
    pub fn filt_wkup_act_feed_get(&mut self) -> Result<FiltWkupActFeed, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self)?;
        let tap_cfg0 = TapCfg0::read(self)?;

        let result =
            FiltWkupActFeed::try_from((wake_up_ths.usr_off_on_wu() << 1) + tap_cfg0.slope_fds())
                .unwrap_or_default();

        Ok(result)
    }
    /// Mask hw function triggers when xl is settling.
    ///
    /// If val is 1 it enables the masking
    pub fn mask_trigger_xl_settl_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_hw_func_mask_xl_settl(val & 0x01);
        tap_cfg0.write(self)
    }
    /// Get current configuration (enable/disable) of mask hw function
    ///
    /// triggers when xl is settling.
    pub fn mask_trigger_xl_settl_get(&mut self) -> Result<u8, Error<B::Error>> {
        TapCfg0::read(self).map(|reg| reg.hw_func_mask_xl_settl())
    }
    /// Configure the LPF2 filter on 6D (sixd) function.
    pub fn filt_sixd_feed_set(&mut self, val: FiltSixdFeed) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_low_pass_on_6d((val as u8) & 0x01);
        tap_cfg0.write(self)
    }
    /// Get the actual LPF2 filter on 6D (sixd) function selection.
    pub fn filt_sixd_feed_get(&mut self) -> Result<FiltSixdFeed, Error<B::Error>> {
        let reg = TapCfg0::read(self)?;
        let val = FiltSixdFeed::try_from(reg.low_pass_on_6d()).unwrap_or_default();
        Ok(val)
    }
    /// Set the gyroscope digital LPF_EIS filter bandwidth.
    pub fn filt_gy_eis_lp_bandwidth_set(
        &mut self,
        val: FiltGyEisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self)?;
        ctrl_eis.set_lpf_g_eis_bw(val as u8 & 0x01);
        ctrl_eis.write(self)
    }
    /// Get the current gyroscope digital LPF_EIS filter bandwidth selection.
    pub fn filt_gy_eis_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltGyEisLpBandwidth, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self)?;

        let val = FiltGyEisLpBandwidth::try_from(ctrl_eis.lpf_g_eis_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Gyroscope OIS digital LPF1 filter bandwidth.
    ///
    /// This function works also on OIS interface.
    pub fn filt_gy_ois_lp_bandwidth_set(
        &mut self,
        val: FiltGyOisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl2_ois = UiCtrl2Ois::read(self)?;

        ui_ctrl2_ois.set_lpf1_g_ois_bw((val as u8) & 0x03);
        ui_ctrl2_ois.write(self)
    }
    /// Get the current gyroscope OIS digital LPF1 filter bandwidth.
    ///
    /// This function works also on OIS interface.
    pub fn filt_gy_ois_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltGyOisLpBandwidth, Error<B::Error>> {
        let ui_ctrl2_ois = UiCtrl2Ois::read(self)?;

        let val = FiltGyOisLpBandwidth::try_from(ui_ctrl2_ois.lpf1_g_ois_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Set accelerometer OIS channel bandwidth.
    ///
    /// This function works also on OIS interface.
    pub fn filt_xl_ois_lp_bandwidth_set(
        &mut self,
        val: FiltXlOisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl3_ois = UiCtrl3Ois::read(self)?;
        ui_ctrl3_ois.set_lpf_xl_ois_bw((val as u8) & 0x07);
        ui_ctrl3_ois.write(self)
    }
    /// Get accelerometer OIS channel bandwidth.
    ///
    /// This function works also on OIS interface.
    pub fn filt_xl_ois_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltXlOisLpBandwidth, Error<B::Error>> {
        let reg = UiCtrl3Ois::read(self)?;

        let val = FiltXlOisLpBandwidth::try_from(reg.lpf_xl_ois_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Enables the control of the CTRL registers to FSM.
    ///
    /// Warning: FSM can change some configurations of the device autonomously.
    pub fn fsm_permission_set(&mut self, val: FsmPermission) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self)?;
        func_cfg_access.set_fsm_wr_ctrl_en(val as u8 & 0x01);
        func_cfg_access.write(self)
    }
    /// Get the FSM permission to change the CTRL registers.
    pub fn fsm_permission_get(&mut self) -> Result<FsmPermission, Error<B::Error>> {
        let func_cfg_access = FuncCfgAccess::read(self)?;

        match func_cfg_access.fsm_wr_ctrl_en() {
            0 => Ok(FsmPermission::ProtectCtrlRegs),
            1 => Ok(FsmPermission::WriteCtrlReg),
            _ => Ok(FsmPermission::ProtectCtrlRegs),
        }
    }
    /// Get the FSM permission status
    ///
    /// Returns 0 if all registers writable from standard interface;
    /// 1 if some registers are under FSM control.
    pub fn fsm_permission_status(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlStatus::read(self).map(|reg| reg.fsm_wr_ctrl_status())?;

        Ok(val)
    }
    /// Enable Finite State Machine (FSM) feature.
    pub fn fsm_mode_set(&mut self, val: FsmMode) -> Result<(), Error<B::Error>> {
        const PROPERTY_ENABLE: u8 = 1;
        const PROPERTY_DISABLE: u8 = 0;

        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state)?;
            let mut fsm_enable = FsmEnable::read(state)?;

            if (val.fsm1_en
                | val.fsm2_en
                | val.fsm3_en
                | val.fsm4_en
                | val.fsm5_en
                | val.fsm6_en
                | val.fsm7_en
                | val.fsm8_en)
                == PROPERTY_ENABLE
            {
                emb_func_en_b.set_fsm_en(PROPERTY_ENABLE);
            } else {
                emb_func_en_b.set_fsm_en(PROPERTY_DISABLE);
            }

            fsm_enable.set_fsm1_en(val.fsm1_en);
            fsm_enable.set_fsm2_en(val.fsm2_en);
            fsm_enable.set_fsm3_en(val.fsm3_en);
            fsm_enable.set_fsm4_en(val.fsm4_en);
            fsm_enable.set_fsm5_en(val.fsm5_en);
            fsm_enable.set_fsm6_en(val.fsm6_en);
            fsm_enable.set_fsm7_en(val.fsm7_en);
            fsm_enable.set_fsm8_en(val.fsm8_en);

            fsm_enable.write(state)?;
            emb_func_en_b.write(state)
        })
    }
    /// Enable Finite State Machine (FSM) feature.
    pub fn fsm_mode_get(&mut self) -> Result<FsmMode, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let fsm_enable = FsmEnable::read(state)?;
            Ok(FsmMode {
                fsm1_en: fsm_enable.fsm1_en(),
                fsm2_en: fsm_enable.fsm2_en(),
                fsm3_en: fsm_enable.fsm3_en(),
                fsm4_en: fsm_enable.fsm4_en(),
                fsm5_en: fsm_enable.fsm5_en(),
                fsm6_en: fsm_enable.fsm6_en(),
                fsm7_en: fsm_enable.fsm7_en(),
                fsm8_en: fsm_enable.fsm8_en(),
            })
        })
    }
    /// Set FSM long counter status register.
    ///
    /// Long counter value is an unsigned integer value (16-bit format).
    pub fn fsm_long_cnt_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| FsmLongCounter(val).write(state))
    }
    /// Get FSM long counter status register.
    ///
    /// Long counter value is an unsigned integer value (16-bit format).
    pub fn fsm_long_cnt_get(&mut self) -> Result<u16, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| FsmLongCounter::read(state).map(|reg| reg.0))
    }
    /// Get the FSM output results.
    pub fn fsm_out_get(&mut self) -> Result<[FsmOutsElement; 8], Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| FsmOut::read(state).map(|reg| reg.0))
    }
    /// Set the Finite State Machine Output Data Rate (ODR).
    pub fn fsm_data_rate_set(&mut self, val: FsmDataRate) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut fsm_odr = FsmOdr::read(state)?;
            fsm_odr.set_fsm_odr((val as u8) & 0x07);
            fsm_odr.write(state)
        })
    }
    /// Get the finite State Machine Output Data Rate (ODR) configuration.
    pub fn fsm_data_rate_get(&mut self) -> Result<FsmDataRate, Error<B::Error>> {
        // Set the memory bank to EmbedFuncMemBank
        MemBank::operate_over_embed(self, |state| {
            // Read the FSM ODR register
            let fsm_odr = FsmOdr::read(state)?;
            // Determine the FSM data rate
            let val = FsmDataRate::try_from(fsm_odr.fsm_odr()).unwrap_or_default();
            Ok(val)
        })
    }
    /// Set SFLP GBIAS value for x/y/z axis.
    ///
    /// The register value is expressed as half-precision
    /// floating-point format: SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent
    /// bits; F: 10 fraction bits).
    pub fn sflp_game_gbias_set(&mut self, val: &SflpGbias) -> Result<(), Error<B::Error>> {
        let mut emb_func_en_saved: (EmbFuncEnA, EmbFuncEnB);
        let mut gbias_hf: [u16; 3] = [0; 3];
        let mut data_tmp: i32;
        let mut data_bytes: [u8; 4];

        let sflp_odr = self.sflp_data_rate_get()?;

        let k = match sflp_odr {
            SflpDataRate::_15hz => 0.04,
            SflpDataRate::_30hz => 0.02,
            SflpDataRate::_60hz => 0.01,
            SflpDataRate::_120hz => 0.005,
            SflpDataRate::_240hz => 0.0025,
            SflpDataRate::_480hz => 0.00125,
        };

        // compute gbias as half precision float in order to be put in embedded advanced feature register
        gbias_hf[0] = self.npy_float_to_half(val.gbias_x * (core::f32::consts::PI / 180.0) / k);
        gbias_hf[1] = self.npy_float_to_half(val.gbias_y * (core::f32::consts::PI / 180.0) / k);
        gbias_hf[2] = self.npy_float_to_half(val.gbias_z * (core::f32::consts::PI / 180.0) / k);

        // Save sensor configuration and set high-performance mode (if the sensor is in the power-down
        // mode, turn it on)
        let conf_saved = (Ctrl1::read(self)?, Ctrl2::read(self)?);
        self.xl_mode_set(XlMode::HighPerformanceMd)?;
        self.gy_mode_set(GyMode::HighPerformanceMd)?;
        if conf_saved.0.odr_xl() == Odr::Off as u8 {
            self.xl_data_rate_set(Odr::_120hz)?;
        }

        // make sure to turn the sensor-hub master off
        let master_config = self.sh_master_get()?;
        self.sh_master_set(0)?;

        // disable algos
        emb_func_en_saved = MemBank::operate_over_embed(self, |state| {
            let tmp_emb_func_en_saved = (EmbFuncEnA::read(state)?, EmbFuncEnB::read(state)?);
            EmbFuncEnA::from_bits(0).write(state)?;
            EmbFuncEnB::from_bits(0).write(state)?;
            loop {
                let emb_func_sts = EmbFuncExecStatus::read(state)?;
                if emb_func_sts.emb_func_endop() == 1 {
                    break;
                }
            }

            Ok(tmp_emb_func_en_saved)
        })?;

        // enable gbias setting
        let mut ctrl10 = Ctrl10::read(self)?;
        ctrl10.set_emb_func_debug(1);
        ctrl10.write(self)?;

        // enable algos
        MemBank::operate_over_embed(self, |state| {
            emb_func_en_saved.0.set_sflp_game_en(1); // force SFLP GAME en
            emb_func_en_saved.0.write(state)?;
            emb_func_en_saved.1.write(state)
        })?;

        let xl_fs = self.xl_full_scale_get()?;

        loop {
            let drdy = self.flag_data_ready_get()?;
            if drdy.drdy_xl == 1 {
                break;
            }
        }

        let xl_data: [i16; 3] = self.acceleration_raw_get()?;

        // force sflp initialization
        self.mem_bank_set(MemBank::SensorHubMemBank)?;
        for i in 0..3 {
            data_tmp = xl_data[i as usize] as i32;
            data_tmp <<= xl_fs as i32; // shift based on current fs
            data_bytes = data_tmp.to_le_bytes();
            self.write_to_register(SnsHubReg::SensorHub1 as u8 + 3 * i, &[data_bytes[0]])?;
            self.write_to_register(SnsHubReg::SensorHub2 as u8 + 3 * i, &[data_bytes[1]])?;
            self.write_to_register(SnsHubReg::SensorHub3 as u8 + 3 * i, &[data_bytes[2]])?;
        }
        for i in 0..3 {
            data_tmp = 0;
            data_bytes = data_tmp.to_le_bytes();
            self.write_to_register(SnsHubReg::SensorHub10 as u8 + 3 * i, &[data_bytes[0]])?;
            self.write_to_register(SnsHubReg::SensorHub11 as u8 + 3 * i, &[data_bytes[1]])?;
            self.write_to_register(SnsHubReg::SensorHub12 as u8 + 3 * i, &[data_bytes[2]])?;
        }
        self.mem_bank_set(MemBank::MainMemBank)?;

        // wait end op (and at least 30 us)
        self.tim.delay_ms(1);
        MemBank::operate_over_embed(self, |state| {
            loop {
                let emb_func_sts = EmbFuncExecStatus::read(state)?;
                if emb_func_sts.emb_func_endop() == 1 {
                    break;
                }
            }
            Ok(())
        })?;

        // write gbias in embedded advanced features registers
        SflpGameGbiasXYZ(gbias_hf).write(self)?;

        // reload previous sensor configuration
        conf_saved.0.write(self)?;
        conf_saved.1.write(self)?;

        // disable gbias setting
        ctrl10.set_emb_func_debug(0);
        ctrl10.write(self)?;

        // reload previous master configuration
        self.sh_master_set(master_config)
    }
    /// Set the External sensor sensitivity value register for the Finite State Machine.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x1624 (when using an external magnetometer this value
    /// corresponds to 0.0015 gauss/LSB).
    pub fn fsm_ext_sens_sensitivity_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmExtSensitivity(val).write(self)
    }
    /// Get the External sensor sensitivity value register for the Finite State Machine.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x1624 (when using an external magnetometer this value
    /// corresponds to 0.0015 gauss/LSB).
    pub fn fsm_ext_sens_sensitivity_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmExtSensitivity::read(self).map(|reg| reg.0)
    }
    /// Set External sensor offsets (X,Y,Z).
    ///
    /// The values are expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub fn fsm_ext_sens_offset_set(
        &mut self,
        val: XlFsmExtSensOffset,
    ) -> Result<(), Error<B::Error>> {
        FsmExtOffXYZ([val.x, val.y, val.z]).write(self)
    }
    /// Get External sensor offsets (X,Y,Z).
    ///
    /// The values are expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub fn fsm_ext_sens_offset_get(&mut self) -> Result<XlFsmExtSensOffset, Error<B::Error>> {
        FsmExtOffXYZ::read(self).map(|reg| XlFsmExtSensOffset {
            x: reg.0[0],
            y: reg.0[1],
            z: reg.0[2],
        })
    }
    /// Set External sensor transformation matrix.
    ///
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub fn fsm_ext_sens_matrix_set(
        &mut self,
        val: XlFsmExtSensMatrix,
    ) -> Result<(), Error<B::Error>> {
        let buff: [u8; 12] = val.to_le_bytes();

        FsmExtMatrix(buff).write(self)
    }
    /// Get the External sensor transformation matrix.
    ///
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub fn fsm_ext_sens_matrix_get(&mut self) -> Result<XlFsmExtSensMatrix, Error<B::Error>> {
        let buff: [u8; 12] = FsmExtMatrix::read(self)?.0;
        Ok(XlFsmExtSensMatrix::from_le_bytes(buff))
    }
    /// Set External sensor z-axis coordinates rotation.
    pub fn fsm_ext_sens_z_orient_set(
        &mut self,
        val: FsmExtSensZOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_a = ExtCfgA::read(self)?;
        ext_cfg_a.set_ext_z_axis((val as u8) & 0x07);
        ext_cfg_a.write(self)
    }
    /// Get External sensor z-axis coordinates rotation.
    pub fn fsm_ext_sens_z_orient_get(&mut self) -> Result<FsmExtSensZOrient, Error<B::Error>> {
        let ext_cfg_a = ExtCfgA::read(self)?;

        let orientation = match ext_cfg_a.ext_z_axis() {
            0x0 => FsmExtSensZOrient::ZEqY,
            0x1 => FsmExtSensZOrient::ZEqMinY,
            0x2 => FsmExtSensZOrient::ZEqX,
            0x3 => FsmExtSensZOrient::ZEqMinX,
            0x4 => FsmExtSensZOrient::ZEqMinZ,
            0x5 => FsmExtSensZOrient::ZEqZ,
            _ => FsmExtSensZOrient::ZEqY, // Default case
        };

        Ok(orientation)
    }
    /// Set External sensor Y-axis coordinates rotation.
    pub fn fsm_ext_sens_y_orient_set(
        &mut self,
        val: FsmExtSensYOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_a = ExtCfgA::read(self)?;
        ext_cfg_a.set_ext_y_axis((val as u8) & 0x7);
        ext_cfg_a.write(self)
    }
    /// Get External sensor Y-axis coordinates rotation.
    pub fn fsm_ext_sens_y_orient_get(&mut self) -> Result<FsmExtSensYOrient, Error<B::Error>> {
        let ext_cfg_a = ExtCfgA::read(self)?;
        let val = FsmExtSensYOrient::try_from(ext_cfg_a.ext_y_axis()).unwrap_or_default();

        Ok(val)
    }
    /// Set External sensor X-axis coordinates rotation.
    pub fn fsm_ext_sens_x_orient_set(
        &mut self,
        val: FsmExtSensXOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_b = ExtCfgB::read(self)?;
        ext_cfg_b.set_ext_x_axis((val as u8) & 0x7);
        ext_cfg_b.write(self)
    }
    /// Get External sensor X-axis coordinates rotation.
    pub fn fsm_ext_sens_x_orient_get(&mut self) -> Result<FsmExtSensXOrient, Error<B::Error>> {
        let ext_cfg_b = ExtCfgB::read(self)?;
        let val = FsmExtSensXOrient::try_from(ext_cfg_b.ext_x_axis()).unwrap_or_default();

        Ok(val)
    }
    /// Set FSM long counter timeout.
    ///
    /// The long counter timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reaches this value, the FSM generates an interrupt.
    pub fn fsm_long_cnt_timeout_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmLcTimeout(val).write(self)
    }
    /// Get FSM long counter timeout.
    ///
    /// The long counter timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reached this value, the FSM generates an interrupt.
    pub fn fsm_long_cnt_timeout_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmLcTimeout::read(self).map(|reg| reg.0)
    }
    /// Set the FSM number of programs.
    ///
    /// Must be less than or equal to 8. Default 0.
    pub fn fsm_number_of_programs_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fsm_programs = FsmPrograms::read(self)?;
        fsm_programs.set_fsm_n_prog(val);
        fsm_programs.write(self)
    }
    /// Get the actual FSM number of programs.
    pub fn fsm_number_of_programs_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FsmPrograms::read(self)?.fsm_n_prog();
        Ok(val)
    }
    /// Set the FSM start address.
    ///
    /// First available address is 0x35C.
    pub fn fsm_start_address_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmStartAdd(val).write(self)
    }
    /// Get the actual FSM start address.
    ///
    /// First available address is 0x35C.
    pub fn fsm_start_address_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmStartAdd::read(self).map(|reg| reg.0)
    }
    /// Set the time windows configuration for Free Fall detection.
    ///
    /// 1 LSB = 1/ODR_XL time
    pub fn ff_time_windows_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        // Read WAKE_UP_DUR and configure wake_up_dur
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_ff_dur((val & 0x20) >> 5);
        wake_up_dur.write(self)?;

        // Read FREE_FALL and configure free_fall
        let mut free_fall = FreeFall::read(self)?;
        free_fall.set_ff_dur(val & 0x1F);
        free_fall.write(self)
    }
    /// Get the time windows configuration for Free Fall detection.
    ///
    /// 1 LSB = 1/ODR_XL time
    pub fn ff_time_windows_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self)?;
        let free_fall = FreeFall::read(self)?;

        let val: u8 = (wake_up_dur.ff_dur() << 5) + free_fall.ff_dur();

        Ok(val)
    }
    /// Set the Free fall threshold.
    pub fn ff_thresholds_set(&mut self, val: FfThreshold) -> Result<(), Error<B::Error>> {
        let mut free_fall = FreeFall::read(self)?;
        free_fall.set_ff_ths((val as u8) & 0x7);

        free_fall.write(self)
    }
    /// Get the current Free fall threshold setting.
    pub fn ff_thresholds_get(&mut self) -> Result<FfThreshold, Error<B::Error>> {
        let free_fall = FreeFall::read(self)?;

        let val = FfThreshold::try_from(free_fall.ff_ths()).unwrap_or_default();

        Ok(val)
    }
    /// Set Machine Learning Core mode (MLC).
    ///registermod
    /// When the Machine Learning Core is enabled the Finite State Machine (FSM)
    /// programs are executed before executing the MLC algorithms.
    pub fn mlc_set(&mut self, val: MlcMode) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_en_a = EmbFuncEnA::read(state)?;
            let mut emb_en_b = EmbFuncEnB::read(state)?;

            match val {
                MlcMode::Off => {
                    emb_en_a.set_mlc_before_fsm_en(0);
                    emb_en_b.set_mlc_en(0);
                }
                MlcMode::On => {
                    emb_en_a.set_mlc_before_fsm_en(0);
                    emb_en_b.set_mlc_en(1);
                }
                MlcMode::OnBeforeFsm => {
                    emb_en_a.set_mlc_before_fsm_en(1);
                    emb_en_b.set_mlc_en(0);
                }
            }

            emb_en_a.write(state)?;
            emb_en_b.write(state)
        })
    }
    /// Get the configuration ofMachine Learning Core (MLC).
    ///
    /// When the Machine Learning Core is enabled the Finite State Machine (FSM)
    /// programs are executed before executing the MLC algorithms.
    pub fn mlc_get(&mut self) -> Result<MlcMode, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_en_a = EmbFuncEnA::read(state)?;
            let emb_en_b = EmbFuncEnB::read(state)?;

            let val = match (emb_en_a.mlc_before_fsm_en(), emb_en_b.mlc_en()) {
                (0, 0) => MlcMode::Off,
                (0, 1) => MlcMode::On,
                (1, _) => MlcMode::OnBeforeFsm,
                _ => MlcMode::Off,
            };

            Ok(val)
        })
    }
    /// Set Machine Learning Core Output Data Rate (ODR).
    pub fn mlc_data_rate_set(&mut self, val: MlcDataRate) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut mlc_odr = MlcOdr::read(state)?;
            mlc_odr.set_mlc_odr((val as u8) & 0x07);
            mlc_odr.write(state)
        })
    }
    /// Get the Machine Learning Core Output Data Rate (ODR).
    pub fn mlc_data_rate_get(&mut self) -> Result<MlcDataRate, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mlc_odr = MlcOdr::read(state)?;
            let val = MlcDataRate::try_from(mlc_odr.mlc_odr()).unwrap_or_default();
            Ok(val)
        })
    }
    /// Get the output value of all MLC decision trees.
    pub fn mlc_out_get(&mut self) -> Result<MlcOut, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mlc_outs = MlcSrc::read(state)?.0;
            Ok(MlcOut::from_le_bytes(mlc_outs))
        })
    }
    /// Set the External sensor sensitivity value register for the Machine Learning Core.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x3C00 (when using an external magnetometer this value
    /// corresponds to 1 gauss/LSB).
    pub fn mlc_ext_sens_sensitivity_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        MlcExtSensitivity(val).write(self)
    }
    /// Get the External sensor sensitivity value register for the Machine Learning Core.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x3C00 (when using an external magnetometer this value
    /// corresponds to 1 gauss/LSB).
    pub fn mlc_ext_sens_sensitivity_get(&mut self) -> Result<u16, Error<B::Error>> {
        MlcExtSensitivity::read(self).map(|reg| reg.0)
    }
    /// Get the configuration for the full control of OIS configurations from the UI (User Interface).
    pub fn ois_ctrl_mode_set(&mut self, val: OisCtrlMode) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self)?;
        func_cfg_access.set_ois_ctrl_from_ui((val as u8) & 0x1);
        func_cfg_access.write(self)
    }
    /// Get the configuration for the full control of OIS configurations from the UI (User Interface).
    pub fn ois_ctrl_mode_get(&mut self) -> Result<OisCtrlMode, Error<B::Error>> {
        let func_cfg_access = FuncCfgAccess::read(self)?;

        let val = OisCtrlMode::try_from(func_cfg_access.ois_ctrl_from_ui()).unwrap_or_default();

        Ok(val)
    }
    /// Resets the control registers of OIS from the UI (User Interface).
    ///
    /// The bits set are not auto-cleared.
    pub fn ois_reset_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self)?;

        func_cfg_access.set_spi2_reset(val as u8);

        func_cfg_access.write(self)
    }
    /// Get Resets the control registers of OIS from the UI (User Interface)
    pub fn ois_reset_get(&mut self) -> Result<i8, Error<B::Error>> {
        let val: i8 = FuncCfgAccess::read(self).map(|reg| reg.spi2_reset() as i8)?;

        Ok(val)
    }
    /// Enable/disable pull up on OIS interface.
    pub fn ois_interface_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self)?;
        pin_ctrl.set_ois_pu_dis(val);
        pin_ctrl.write(self)
    }
    /// Get the configuration (enable/disable) pull up on OIS interface.
    pub fn ois_interface_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PinCtrl::read(self).map(|reg| reg.ois_pu_dis())?;

        Ok(val)
    }
    /// Set Handshake for UI (User Interface) SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake. If the secondary interface is not accessing the shared registers,
    /// this bit is set to 1 by the device and the R/W operation on the UI_SPI2_SHARED registers is allowed
    /// on the primary interface.
    ///
    /// REQ: This bit is used by the primary interface master to request access to the UI_SPI2_SHARED registers.
    /// When the R/W operation is finished, the master must reset this bit.
    pub fn ois_handshake_from_ui_set(&mut self, val: OisHandshake) -> Result<(), Error<B::Error>> {
        let mut ui_handshake_ctrl = UiHandshakeCtrl::read(self)?;
        ui_handshake_ctrl.set_ui_shared_ack(val.ack);
        ui_handshake_ctrl.set_ui_shared_req(val.req);
        ui_handshake_ctrl.write(self)
    }
    /// Get Handshake for UI (User Interface) SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake.
    /// If the secondary interface is not accessing the shared registers, this bit is set to 1 by the device and the R/W operation
    /// on the UI_SPI2_SHARED registers is allowed on the primary interface. REQ: This bit is used by the primary interface master
    /// to request access to the UI_SPI2_SHARED registers. When the R/W operation is finished, the master must reset this bit.
    pub fn ois_handshake_from_ui_get(&mut self) -> Result<OisHandshake, Error<B::Error>> {
        let ui_handshake_ctrl: UiHandshakeCtrl = UiHandshakeCtrl::read(self)?;
        let value = OisHandshake {
            ack: ui_handshake_ctrl.ui_shared_ack(),
            req: ui_handshake_ctrl.ui_shared_req(),
        };
        Ok(value)
    }
    /// Set Handshake for OIS interface SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake. If the secondary interface is not accessing the shared registers,
    /// this bit is set to 1 by the device and the R/W operation on the UI_SPI2_SHARED registers is allowed on the primary interface.
    /// REQ: This bit is used by the primary interface master to request access to the UI_SPI2_SHARED registers.
    /// When the R/W operation is finished, the master must reset this bit.
    pub fn ois_handshake_from_ois_set(&mut self, val: OisHandshake) -> Result<(), Error<B::Error>> {
        let mut spi2_handshake_ctrl: Spi2HandshakeCtrl = Spi2HandshakeCtrl::read(self)?;
        spi2_handshake_ctrl.set_spi2_shared_ack(val.ack);
        spi2_handshake_ctrl.set_spi2_shared_req(val.req);
        spi2_handshake_ctrl.write(self)
    }
    /// Get Handshake for OIS interface SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake. If the secondary interface is not accessing
    /// the shared registers, this bit is set to 1 by the device and the R/W operation on the
    /// UI_SPI2_SHARED registers is allowed on the primary interface.
    /// REQ: This bit is used by the primary interface master to request access to the
    /// UI_SPI2_SHARED registers. When the R/W operation is finished, the master must reset this bit.
    pub fn ois_handshake_from_ois_get(&mut self) -> Result<OisHandshake, Error<B::Error>> {
        let spi2_handshake_ctrl = Spi2HandshakeCtrl::read(self)?;
        let val = OisHandshake {
            ack: spi2_handshake_ctrl.spi2_shared_ack(),
            req: spi2_handshake_ctrl.spi2_shared_req(),
        };

        Ok(val)
    }
    /// Set User interface (UI) / SPI2 (OIS) shared registers
    pub fn ois_shared_set(&mut self, val: [u8; 6]) -> Result<(), Error<B::Error>> {
        UiSpi2Shared(val).write(self)
    }
    /// Get User interface (UI) / SPI2 (OIS) shared registers
    pub fn ois_shared_get(&mut self) -> Result<[u8; 6], Error<B::Error>> {
        UiSpi2Shared::read(self).map(|reg| reg.0)
    }
    /// Enables SPI2 (OIS Interface) for reading OIS data when User Interface (UI) is in full control mode
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub fn ois_on_spi2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self)?;
        ui_ctrl1_ois.set_spi2_read_en(val);
        ui_ctrl1_ois.write(self)
    }
    /// Get configuration (enable/disable) of SPI2 (OIS Interface) for reading OIS data when
    /// User Interface (UI) is in full control mode
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub fn ois_on_spi2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = UiCtrl1Ois::read(self).map(|reg| reg.spi2_read_en())?;

        Ok(val)
    }
    /// Enables gyroscope/accelerometer OIS chain.
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub fn ois_chain_set(&mut self, val: OisChain) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self)?;
        ui_ctrl1_ois.set_ois_g_en(val.gy);
        ui_ctrl1_ois.set_ois_xl_en(val.xl);
        ui_ctrl1_ois.write(self)
    }
    /// Get the configuration (enable/disable) gyroscope/accelerometer OIS chain.
    pub fn ois_chain_get(&mut self) -> Result<OisChain, Error<B::Error>> {
        let ui_ctrl1_ois = UiCtrl1Ois::read(self)?;
        let gy = ui_ctrl1_ois.ois_g_en();
        let xl = ui_ctrl1_ois.ois_xl_en();

        let val = OisChain { gy, xl };

        Ok(val)
    }
    /// Set gyroscope OIS full-scale
    pub fn ois_gy_full_scale_set(&mut self, val: OisGyFullScale) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl2_ois = UiCtrl2Ois::read(self)?;
        ui_ctrl2_ois.set_fs_g_ois(val as u8 & 0x03);
        ui_ctrl2_ois.write(self)
    }
    /// Get the gyroscope OIS full-scale selection
    pub fn ois_gy_full_scale_get(&mut self) -> Result<OisGyFullScale, Error<B::Error>> {
        let fs_g_ois = UiCtrl2Ois::read(self).map(|reg| reg.fs_g_ois())?;

        let val = OisGyFullScale::try_from(fs_g_ois).unwrap_or_default();

        Ok(val)
    }
    /// Set accelerometer OIS channel full-scale.
    pub fn ois_xl_full_scale_set(&mut self, val: OisXlFullScale) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl3_ois = UiCtrl3Ois::read(self)?;
        ui_ctrl3_ois.set_fs_xl_ois((val as u8) & 0x3);
        ui_ctrl3_ois.write(self)
    }
    /// Get accelerometer OIS channel full-scale.
    pub fn ois_xl_full_scale_get(&mut self) -> Result<OisXlFullScale, Error<B::Error>> {
        let ui_ctrl3_ois = UiCtrl3Ois::read(self)?;

        let val = OisXlFullScale::try_from(ui_ctrl3_ois.fs_xl_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Set Threshold for 4D/6D function.
    pub fn threshold_6d_set(&mut self, val: SixDThreshold) -> Result<(), Error<B::Error>> {
        let mut tap_ths_6d = TapThs6d::read(self)?;
        tap_ths_6d.set_sixd_ths((val as u8) & 0x03);
        tap_ths_6d.write(self)
    }
    /// Get Threshold for 4D/6D function.
    pub fn threshold_6d_get(&mut self) -> Result<SixDThreshold, Error<B::Error>> {
        let tap_ths_6d = TapThs6d::read(self)?;

        let value = SixDThreshold::try_from(tap_ths_6d.sixd_ths()).unwrap_or_default();

        Ok(value)
    }
    /// Enables/Disables 4D orientation detection.
    ///
    /// Z-axis position detection is disabled.
    pub fn mode_4d_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = TapThs6d::read(self)?;
        reg.set_d4d_en(val);
        reg.write(self)
    }
    /// Get the configuration for 4D orientation detection enable.
    ///
    /// Z-axis position detection is disabled.
    pub fn mode_4d_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = TapThs6d::read(self).map(|reg| reg.d4d_en())?;

        Ok(val)
    }
    /// Configures the equivalent input impedance of the AH_QVAR buffers.
    pub fn ah_qvar_zin_set(&mut self, val: AhQvarZin) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self)?;
        ctrl7.set_ah_qvar_c_zin((val as u8) & 0x03);
        ctrl7.write(self)
    }
    /// Get the actual input impedance of the AH_QVAR buffers.
    pub fn ah_qvar_zin_get(&mut self) -> Result<AhQvarZin, Error<B::Error>> {
        let ctrl7 = Ctrl7::read(self)?;

        let result = AhQvarZin::try_from(ctrl7.ah_qvar_c_zin()).unwrap_or_default();

        Ok(result)
    }
    /// Enables AH_QVAR chain.
    ///
    /// When this bit is set to '1', the AH_QVAR buffers are connected to the SDx/AH1/Qvar1 and SCx/AH2/Qvar2 pins.
    /// Before setting this bit to 1, the accelerometer and gyroscope sensor have to be configured in power-down mode.
    pub fn ah_qvar_mode_set(&mut self, val: AhQvarMode) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self)?;
        ctrl7.set_ah_qvar_en(val.ah_qvar_en);
        ctrl7.write(self)
    }
    /// Get configuration (enable/disable) for AH_QVAR chain.
    ///
    /// When this bit is set to '1', the AH_QVAR buffers are connected to the SDx/AH1/Qvar1 and SCx/AH2/Qvar2 pins.
    /// Before setting this bit to 1, the accelerometer and gyroscope sensor have to be configured in power-down mode.
    pub fn ah_qvar_mode_get(&mut self) -> Result<AhQvarMode, Error<B::Error>> {
        let ctrl7 = Ctrl7::read(self)?;
        let val = AhQvarMode {
            ah_qvar_en: ctrl7.ah_qvar_en(),
        };

        Ok(val)
    }
    /// Set the action the device will perform after "Reset whole chip" I3C pattern.
    pub fn i3c_reset_mode_set(&mut self, val: I3cResetMode) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self)?;
        pin_ctrl.set_ibhr_por_en((val as u8) & 0x01);
        pin_ctrl.write(self)
    }
    /// Get the i3c reset mode.
    ///
    /// After set the action the device will perform after "Reset whole chip" I3C pattern.
    pub fn i3c_reset_mode_get(&mut self) -> Result<I3cResetMode, Error<B::Error>> {
        let pin_ctrl = PinCtrl::read(self)?;
        let mode = I3cResetMode::try_from(pin_ctrl.ibhr_por_en()).unwrap_or_default();
        Ok(mode)
    }
    /// Enable/Disable INT pin when I3C is used.
    pub fn i3c_int_en_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl5 = Ctrl5::read(self)?;
        ctrl5.set_int_en_i3c(val & 0x01);
        ctrl5.write(self)
    }
    /// Get configuration (enable/disable) INT pin when I3C is used.
    pub fn i3c_int_en_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl5::read(self).map(|reg| reg.int_en_i3c())?;

        Ok(val)
    }
    /// Set the us activity time for IBI (In-Band Interrupt) with I3C.
    pub fn i3c_ibi_time_set(&mut self, val: I3cIbiTime) -> Result<(), Error<B::Error>> {
        let mut ctrl5 = Ctrl5::read(self)?;
        ctrl5.set_bus_act_sel((val as u8) & 0x03);
        ctrl5.write(self)
    }
    /// Get the us activity time for IBI (In-Band Interrupt) with I3C.
    pub fn i3c_ibi_time_get(&mut self) -> Result<I3cIbiTime, Error<B::Error>> {
        let ctrl5 = Ctrl5::read(self)?;

        let val = I3cIbiTime::try_from(ctrl5.bus_act_sel()).unwrap_or_default();

        Ok(val)
    }
    /// Enable/Disable Sensor Hub master I2C pull-up.
    pub fn sh_master_interface_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self)?;
        if_cfg.set_shub_pu_en(val);
        if_cfg.write(self)
    }
    /// Get Sensor Hub master I2C pull-up configuration (enable/disable).
    pub fn sh_master_interface_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = IfCfg::read(self).map(|reg| reg.shub_pu_en())?;

        Ok(val)
    }
    /// Sensor hub output registers.
    ///
    /// The length of the array input determines the length of the read.
    /// Valid range goes from 0..18
    pub fn sh_read_data_raw_get(&mut self, rbuf: &mut [u8]) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| SensorHub1::read_more(state, rbuf))
    }
    /// Set the number of external sensors to be read by the sensor hub.
    pub fn sh_slave_connected_set(&mut self, val: ShSlaveConnected) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_aux_sens_on((val as u8) & 0x3);
            master_config.write(state)
        })
    }
    /// Get the number of external sensors to be read by the sensor hub.
    pub fn sh_slave_connected_get(&mut self) -> Result<ShSlaveConnected, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let master_config = MasterConfig::read(state)?;
            let result =
                ShSlaveConnected::try_from(master_config.aux_sens_on()).unwrap_or_default();
            Ok(result)
        })
    }
    /// Enable/Disable Sensor hub I2C master configuration.
    pub fn sh_master_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_master_on(val);
            master_config.write(state)
        })
    }
    /// Get Sensor hub I2C master configuration (enable/disable).
    pub fn sh_master_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let master_config = MasterConfig::read(state)?;
            let val: u8 = master_config.master_on();
            Ok(val)
        })
    }
    /// Enable/Disable I2C interface pass-through.
    pub fn sh_pass_through_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        // Set the memory bank to SensorHubMemBank
        MemBank::operate_over_sensorhub(self, |state| {
            // Modify the MasterConfig register
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_pass_through_mode(val);
            master_config.write(state)
        })
    }
    /// Get I2C interface pass-through configuration (enable/disable).
    pub fn sh_pass_through_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let master_config = MasterConfig::read(state)?;
            let val: u8 = master_config.pass_through_mode();
            Ok(val)
        })
    }
    /// Set Sensor hub trigger signal configuration.
    pub fn sh_syncro_mode_set(&mut self, val: ShSyncroMode) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_start_config((val as u8) & 0x01);
            master_config.write(state)
        })
    }
    /// Get Sensor hub trigger signal configuration.
    pub fn sh_syncro_mode_get(&mut self) -> Result<ShSyncroMode, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let master_config = MasterConfig::read(state)?;
            let val = ShSyncroMode::try_from(master_config.start_config()).unwrap_or_default();
            Ok(val)
        })
    }
    /// Set Slave 0 write operation mode.
    ///
    /// Permit switch write mode between: only at the first sensor hub cycle or at each sensor hub cycle.
    pub fn sh_write_mode_set(&mut self, val: ShWriteMode) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_write_once(val as u8 & 0x01);
            master_config.write(state)
        })
    }
    /// Get Slave 0 write operation mode.
    pub fn sh_write_mode_get(&mut self) -> Result<ShWriteMode, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let write_once = MasterConfig::read(state)?.write_once();
            let mode = ShWriteMode::try_from(write_once).unwrap_or_default();
            Ok(mode)
        })
    }
    /// Reset Master logic and output registers.
    ///
    /// Must be set to '1' and then set it to '0'.
    pub fn sh_reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut master_config = MasterConfig::read(state)?;
            master_config.set_rst_master_regs(val);
            master_config.write(state)
        })
    }
    /// Get Reset Master logic and output registers.
    ///
    /// Must be set to '1' and then set it to '0'.
    pub fn sh_reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let master_config = MasterConfig::read(state)?;
            Ok(master_config.rst_master_regs())
        })
    }
    /// Configure slave 0 to perform a write.
    pub fn sh_cfg_write(&mut self, val: ShCfgWrite) -> Result<(), Error<B::Error>> {
        // Set the memory bank to SensorHubMemBank
        MemBank::operate_over_sensorhub(self, |state| {
            let mut reg = Slv0Add::read(state)?;
            reg.set_slave0_add(val.slv0_add);
            reg.set_rw_0(0);
            // Write to the Slv0Add register
            reg.write(state)?;

            // Write to the DatawriteSlv0 register
            let data_write_slv0 = DatawriteSlv0::from_bits(val.slv0_data);
            data_write_slv0.write(state)
        })
    }
    /// Set the rate at which the master communicates.
    pub fn sh_data_rate_set(&mut self, val: ShDataRate) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let mut slv0_config = Slv0Config::read(state)?;

            slv0_config.set_shub_odr((val as u8) & 0x07);
            slv0_config.write(state)
        })
    }
    /// Get the rate at which the controller communicates.
    pub fn sh_data_rate_get(&mut self) -> Result<ShDataRate, Error<B::Error>> {
        MemBank::operate_over_sensorhub(self, |state| {
            let slv0_config = Slv0Config::read(state)?;
            let rate = ShDataRate::try_from(slv0_config.shub_odr()).unwrap_or_default();
            Ok(rate)
        })
    }
    /// Configure slave idx for performing a read.
    ///
    /// # Arguments
    ///
    /// * `idx`: Index of the slave.
    /// * `val`: Structure containing:
    ///    - `slv_add`: 8-bit I2C device address.
    ///    - `slv_subadd`: 8-bit register device address.
    ///    - `slv_len`: Number of bits to read.
    pub fn sh_slv_cfg_read(&mut self, idx: u8, val: &ShCfgRead) -> Result<(), Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank)?;
        let mut arr: [u8; 1] = [0];
        let mut slv_add = Slv0Add::from_bits(arr[0]);
        slv_add.set_slave0_add(val.slv_add);
        slv_add.set_rw_0(1);

        self.write_to_register(SnsHubReg::Slv0Add as u8 + (idx * 3), &[slv_add.into_bits()])?;

        self.write_to_register(SnsHubReg::Slv0Subadd as u8 + (idx * 3), &[val.slv_subadd])?;

        self.read_from_register(SnsHubReg::Slv0Config as u8 + (idx * 3), &mut arr)?;
        let mut slv_config = Slv0Config::from_bits(arr[0]);
        slv_config.set_slave0_numop(val.slv_len);

        self.write_to_register(
            SnsHubReg::Slv0Config as u8 + (idx * 3),
            &[slv_config.into_bits()],
        )?;

        self.mem_bank_set(MemBank::MainMemBank)
    }
    /// Get Sensor hub status register.
    pub fn sh_status_get(&mut self) -> Result<StatusMaster, Error<B::Error>> {
        let status = StatusMasterMainPage::read(self)?;
        Ok(StatusMaster::from_bits(status.into_bits()))
    }
    /// Enables/Disables pull-up on SDO pin of UI (User Interface).
    pub fn ui_sdo_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self)?;
        pin_ctrl.set_sdo_pu_en(val);
        pin_ctrl.write(self)
    }
    /// Get the pull-up on SDO pin of UI (User Interface) configuration.
    pub fn ui_sdo_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PinCtrl::read(self).map(|reg| reg.sdo_pu_en())?;

        Ok(val)
    }
    /// Enables/Disables I2C and I3C on UI (User Interface).
    pub fn ui_i2c_i3c_mode_set(&mut self, val: UiI2cI3cMode) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self)?;
        if_cfg.set_i2c_i3c_disable((val as u8) & 0x1);
        if_cfg.write(self)
    }
    /// Get I2C and I3C on UI (User Interface) mode configuration.
    pub fn ui_i2c_i3c_mode_get(&mut self) -> Result<UiI2cI3cMode, Error<B::Error>> {
        let reg = IfCfg::read(self)?;

        let val = UiI2cI3cMode::try_from(reg.i2c_i3c_disable()).unwrap_or_default();

        Ok(val)
    }
    /// SPI Serial Interface Mode selection.
    pub fn spi_mode_set(&mut self, val: SpiMode) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self)?;
        if_cfg.set_sim((val as u8) & 0x01);
        if_cfg.write(self)
    }
    /// Get the SPI Serial Interface Mode.
    pub fn spi_mode_get(&mut self) -> Result<SpiMode, Error<B::Error>> {
        let if_cfg = IfCfg::read(self)?;

        let val = SpiMode::try_from(if_cfg.sim()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables pull-up on SDA pin.
    pub fn ui_sda_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self)?;
        if_cfg.set_sda_pu_en(val);
        if_cfg.write(self)
    }
    /// Get pull-up configuration on SDA pin.
    pub fn ui_sda_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = IfCfg::read(self).map(|reg| reg.sda_pu_en())?;

        Ok(val)
    }
    /// Select SPI2 (OIS Interface) Serial Interface Mode.
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub fn spi2_mode_set(&mut self, val: Spi2Mode) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self)?;
        ui_ctrl1_ois.set_sim_ois((val as u8) & 0x01);
        ui_ctrl1_ois.write(self)
    }
    /// Get SPI2 (OIS Interface) Serial Interface Mode.
    /// This function also works on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub fn spi2_mode_get(&mut self) -> Result<Spi2Mode, Error<B::Error>> {
        let ui_ctrl1_ois = UiCtrl1Ois::read(self)?;

        let val = Spi2Mode::try_from(ui_ctrl1_ois.sim_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables significant motion detection function.
    pub fn sigmot_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_sign_motion_en(val);
            emb_func_en_a.write(state)
        })
    }
    /// Get significant motion detection function configuration (enable/disable).
    pub fn sigmot_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            EmbFuncEnA::read(state).map(|reg| reg.sign_motion_en())
        })
    }
    /// Set Step counter mode.
    pub fn stpcnt_mode_set(&mut self, val: StpcntMode) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            let emb_func_en_b = EmbFuncEnB::read(state)?;

            if val.false_step_rej == 1
                && emb_func_en_a.mlc_before_fsm_en() & emb_func_en_b.mlc_en() == 0
            {
                emb_func_en_a.set_mlc_before_fsm_en(1);
            }

            emb_func_en_a.set_pedo_en(val.step_counter_enable);
            emb_func_en_a.write(state)
        })?;

        let mut pedo_cmd_reg = PedoCmdReg::read(self)?;
        pedo_cmd_reg.set_fp_rejection_en(val.false_step_rej);
        pedo_cmd_reg.write(self)
    }
    /// Get Step counter mode
    pub fn stpcnt_mode_get(&mut self) -> Result<StpcntMode, Error<B::Error>> {
        let emb_func_en_a = MemBank::operate_over_embed(self, |state| EmbFuncEnA::read(state))?;
        let pedo_cmd_reg = PedoCmdReg::read(self)?;

        let val = StpcntMode {
            false_step_rej: pedo_cmd_reg.fp_rejection_en(),
            step_counter_enable: emb_func_en_a.pedo_en(),
        };

        Ok(val)
    }
    /// Get Step counter output: number of detected steps.
    pub fn stpcnt_steps_get(&mut self) -> Result<u16, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| StepCounter::read(state).map(|reg| reg.0))
    }
    /// Reset step counter.
    ///
    /// If val is 1: step counter is reset
    pub fn stpcnt_rst_step_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_src: EmbFuncSrc = EmbFuncSrc::read(state)?;
            emb_func_src.set_pedo_rst_step(val);
            emb_func_src.write(state)
        })
    }
    /// Get reset step counter.
    pub fn stpcnt_rst_step_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            EmbFuncSrc::read(state).map(|reg| reg.pedo_rst_step())
        })
    }
    /// Set Pedometer debounce number.
    pub fn stpcnt_debounce_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pedo_deb_steps_conf = PedoDebStepsConf::read(self)?;
        pedo_deb_steps_conf.set_deb_step(val);
        pedo_deb_steps_conf.write(self)
    }
    /// Get Pedometer debounce number.
    pub fn stpcnt_debounce_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PedoDebStepsConf::read(self)?.deb_step();

        Ok(val)
    }
    /// Set time period register for step detection on delta time.
    pub fn stpcnt_period_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let reg = PedoScDeltaT(val);
        reg.write(self)
    }
    /// Get time period register for step detection on delta time.
    pub fn stpcnt_period_get(&mut self) -> Result<u16, Error<B::Error>> {
        PedoScDeltaT::read(self).map(|reg| reg.0)
    }
    /// Enables/Disables SFLP Game Rotation Vector (6x).
    pub fn sflp_game_rotation_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_sflp_game_en(val);
            emb_func_en_a.write(state)
        })
    }
    /// Get the configuration (enable/disable) for SFLP Game Rotation Vector (6x).
    pub fn sflp_game_rotation_get(&mut self) -> Result<u8, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let emb_func_en_a = EmbFuncEnA::read(state)?;
            Ok(emb_func_en_a.sflp_game_en())
        })
    }
    /// Set SFLP Data Rate (ODR).
    pub fn sflp_data_rate_set(&mut self, val: SflpDataRate) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut sflp_odr = SflpOdr::read(state)?;
            sflp_odr.set_sflp_game_odr((val as u8) & 0x07);
            sflp_odr.write(state)
        })
    }
    /// Get SFLP Data Rate (ODR).
    pub fn sflp_data_rate_get(&mut self) -> Result<SflpDataRate, Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let sflp = SflpOdr::read(state)?;
            let odr = SflpDataRate::try_from(sflp.sflp_game_odr()).unwrap_or_default();
            Ok(odr)
        })
    }
    /// Enable axis for Tap - Double Tap detection.
    pub fn tap_detection_set(&mut self, val: TapDetection) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self)?;
        tap_cfg0.set_tap_x_en(val.tap_x_en);
        tap_cfg0.set_tap_y_en(val.tap_y_en);
        tap_cfg0.set_tap_z_en(val.tap_z_en);
        tap_cfg0.write(self)
    }
    /// Get configuration for Tap on each axis - Double Tap detection.
    pub fn tap_detection_get(&mut self) -> Result<TapDetection, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self)?;

        let tap_detection = TapDetection {
            tap_x_en: tap_cfg0.tap_x_en(),
            tap_y_en: tap_cfg0.tap_y_en(),
            tap_z_en: tap_cfg0.tap_z_en(),
        };

        Ok(tap_detection)
    }
    /// Set Double Tap recognition thresholds - axis Tap
    pub fn tap_thresholds_set(&mut self, val: TapThresholds) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self)?;
        let mut tap_cfg2 = TapCfg2::read(self)?;
        let mut tap_ths_6d = TapThs6d::read(self)?;

        tap_cfg1.set_tap_ths_x(val.x);
        tap_cfg2.set_tap_ths_y(val.y);
        tap_ths_6d.set_tap_ths_z(val.z);

        tap_ths_6d.write(self)?;
        tap_cfg2.write(self)?;
        tap_cfg1.write(self)
    }
    /// Get Double Tap recognition thresholds - axis Tap
    pub fn tap_thresholds_get(&mut self) -> Result<TapThresholds, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self)?;
        let tap_cfg2 = TapCfg2::read(self)?;
        let tap_ths_6d = TapThs6d::read(self)?;

        let thresholds = TapThresholds {
            // Dummy values or replace with appropriate method calls
            x: tap_cfg1.tap_ths_x(),
            y: tap_cfg2.tap_ths_y(),
            z: tap_ths_6d.tap_ths_z(),
        };

        Ok(thresholds)
    }
    /// Set axis priority for TAP detection.
    pub fn tap_axis_priority_set(&mut self, val: TapAxisPriority) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self)?;
        tap_cfg1.set_tap_priority((val as u8) & 0x7);
        tap_cfg1.write(self)
    }
    /// Get axis priority for TAP detection.
    pub fn tap_axis_priority_get(&mut self) -> Result<TapAxisPriority, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self)?;

        let val = TapAxisPriority::try_from(tap_cfg1.tap_priority()).unwrap_or_default();

        Ok(val)
    }
    /// Set Time windows configuration for Tap - Double Tap.
    ///
    /// SHOCK Maximum duration is the maximum time of an overthreshold
    /// signal detection to be recognized as a tap event.
    /// If the SHOCK bits are set to a different value, 1LSB corresponds to 8/ODR_XL time.
    ///
    /// QUIET Expected quiet time after a tap detection. Quiet time is the time
    /// after the first detected tap in which there must not be any overthreshold event.
    /// If the QUIET bits are set to a different value, 1LSB corresponds to 4/ODR_XL time.
    ///
    /// DUR Duration of maximum time gap for double tap recognition. This register expresses
    /// the maximum time between two consecutive detected taps to determine a double tap event.
    /// If the DUR_[3:0] bits are set to a different value, 1LSB corresponds to 32/ODR_XL time.
    pub fn tap_time_windows_set(&mut self, val: TapTimeWindows) -> Result<(), Error<B::Error>> {
        let mut tap_dur = TapDur::read(self)?;
        tap_dur.set_shock(val.shock);
        tap_dur.set_quiet(val.quiet);
        tap_dur.set_dur(val.tap_gap);
        tap_dur.write(self)
    }
    /// Get Time windows configuration for Tap - Double Tap SHOCK, QUIET, DUR.
    ///
    /// SHOCK Maximum duration is the maximum time of an overthreshold
    /// signal detection to be recognized as a tap event.
    /// If the SHOCK bits are set to a different value, 1LSB corresponds to 8/ODR_XL time.
    ///
    /// QUIET Expected quiet time after a tap detection. Quiet time is the time
    /// after the first detected tap in which there must not be any overthreshold event.
    /// If the QUIET bits are set to a different value, 1LSB corresponds to 4/ODR_XL time.
    ///
    /// DUR Duration of maximum time gap for double tap recognition. This register expresses
    /// the maximum time between two consecutive detected taps to determine a double tap event.
    /// If the DUR_[3:0] bits are set to a different value, 1LSB corresponds to 32/ODR_XL time.
    pub fn tap_time_windows_get(&mut self) -> Result<TapTimeWindows, Error<B::Error>> {
        let tap_dur = TapDur::read(self)?;

        let val = TapTimeWindows {
            shock: tap_dur.shock(),
            quiet: tap_dur.quiet(),
            tap_gap: tap_dur.dur(),
        };

        Ok(val)
    }
    /// Enable/Disable single/double-tap event.
    pub fn tap_mode_set(&mut self, val: TapMode) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self)?;
        wake_up_ths.set_single_double_tap((val as u8) & 0x01);
        wake_up_ths.write(self)
    }
    /// Get configuration (enable/disable) for Single/double-tap event
    pub fn tap_mode_get(&mut self) -> Result<TapMode, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self)?;

        let val = TapMode::try_from(wake_up_ths.single_double_tap()).unwrap_or_default();

        Ok(val)
    }
    /// Set Tilt mode.
    pub fn tilt_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        MemBank::operate_over_embed(self, |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state)?;
            emb_func_en_a.set_tilt_en(val);
            emb_func_en_a.write(state)
        })
    }
    /// Get Tilt mode.
    pub fn tilt_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        // Switch to the embedded function memory bank
        MemBank::operate_over_embed(self, |state| {
            // Read the register values
            EmbFuncEnA::read(state).map(|reg| reg.tilt_en())
            // Switch back to the main memory bank
        })
    }
    /// Get Timestamp raw data
    pub fn timestamp_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        Timestamp::read(self).map(|reg| reg.0)
    }
    /// Enables timestamp counter.
    pub fn timestamp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self)?;
        functions_enable.set_timestamp_en(val);
        functions_enable.write(self)
    }
    /// Get the actual timestamp counter configuration.
    ///
    /// If return 1 timestamp counter is active
    pub fn timestamp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FunctionsEnable::read(self).map(|reg| reg.timestamp_en())?;

        Ok(val)
    }

    /// Configure activity/inactivity (sleep)
    ///
    /// `ActMode` could handle different setting for accelerometer and gyroscope
    pub fn act_mode_set(&mut self, val: ActMode) -> Result<(), Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self)?;
        functions_enable.set_inact_en(val as u8 & 0x3);
        functions_enable.write(self)
    }
    /// Get activity/inactivity (sleep)
    pub fn act_mode_get(&mut self) -> Result<ActMode, Error<B::Error>> {
        let functions_enable = FunctionsEnable::read(self)?;

        let val = ActMode::try_from(functions_enable.inact_en()).unwrap_or_default();

        Ok(val)
    }
    /// Set duration in the transition from Stationary to Motion (from Inactivity to Activity).
    pub fn act_from_sleep_to_act_dur_set(
        &mut self,
        val: ActFromSleepToActDur,
    ) -> Result<(), Error<B::Error>> {
        let mut inactivity_dur = InactivityDur::read(self)?;
        inactivity_dur.set_inact_dur((val as u8) & 0x3);
        inactivity_dur.write(self)
    }
    /// Get duration in the transition from Stationary to Motion (from Inactivity to Activity).
    pub fn act_from_sleep_to_act_dur_get(
        &mut self,
    ) -> Result<ActFromSleepToActDur, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self)?;

        let val = ActFromSleepToActDur::try_from(inactivity_dur.inact_dur()).unwrap_or_default();

        Ok(val)
    }
    /// Set the accelerometer data rate during Inactivity.
    pub fn act_sleep_xl_odr_set(&mut self, val: ActSleepXlOdr) -> Result<(), Error<B::Error>> {
        let mut inactivity_dur = InactivityDur::read(self)?;
        inactivity_dur.set_xl_inact_odr((val as u8) & 0x03);
        inactivity_dur.write(self)
    }
    /// Get the accelerometer data rate during Inactivity.
    pub fn act_sleep_xl_odr_get(&mut self) -> Result<ActSleepXlOdr, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self)?;

        let val = ActSleepXlOdr::try_from(inactivity_dur.xl_inact_odr()).unwrap_or_default();

        Ok(val)
    }
    /// Set Wakeup and activity/inactivity threshold.
    pub fn act_thresholds_set(&mut self, val: ActThresholds) -> Result<(), Error<B::Error>> {
        // Read current values from registers
        let mut inactivity_dur = InactivityDur::read(self)?;
        let mut inactivity_ths = InactivityThs::read(self)?;
        let mut wake_up_ths = WakeUpThs::read(self)?;
        let mut wake_up_dur = WakeUpDur::read(self)?;

        // Set new values
        inactivity_dur.set_wu_inact_ths_w(val.inactivity_cfg.wu_inact_ths_w());
        inactivity_dur.set_xl_inact_odr(val.inactivity_cfg.xl_inact_odr());
        inactivity_dur.set_inact_dur(val.inactivity_cfg.inact_dur());

        inactivity_ths.set_inact_ths(val.inactivity_ths);
        wake_up_ths.set_wk_ths(val.threshold);
        wake_up_dur.set_wake_dur(val.duration);

        // Write new values back to registers
        inactivity_dur.write(self)?;
        inactivity_ths.write(self)?;
        wake_up_ths.write(self)?;
        wake_up_dur.write(self)
    }
    /// Get Wakeup and activity/inactivity threshold.
    pub fn act_thresholds_get(&mut self) -> Result<ActThresholds, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self)?;
        let inactivity_ths = InactivityThs::read(self)?;
        let wake_up_ths = WakeUpThs::read(self)?;
        let wake_up_dur = WakeUpDur::read(self)?;

        let val = ActThresholds {
            inactivity_cfg: inactivity_dur,
            inactivity_ths: inactivity_ths.inact_ths(),
            threshold: wake_up_ths.wk_ths(),
            duration: wake_up_dur.wake_dur(),
        };

        Ok(val)
    }
    /// Set Time windows for Wake Up - Activity - Inactivity (SLEEP, WAKE).
    ///
    /// Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR)
    /// 1 LSB = 512/ODR_XL time. Wake up duration event. 1 LSB = 1/ODR_XL time.
    pub fn act_wkup_time_windows_set(
        &mut self,
        val: ActWkupTimeWindows,
    ) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self)?;
        wake_up_dur.set_wake_dur(val.shock);
        wake_up_dur.set_sleep_dur(val.quiet);
        wake_up_dur.write(self)
    }
    /// Get Time windows configuration for Wake Up - Activity - Inactivity (SLEEP, WAKE).
    ///
    /// Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR)
    /// 1 LSB = 512/ODR_XL time. Wake up duration event. 1 LSB = 1/ODR_XL time.
    pub fn act_wkup_time_windows_get(&mut self) -> Result<ActWkupTimeWindows, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self)?;

        let val = ActWkupTimeWindows {
            shock: wake_up_dur.wake_dur(),
            quiet: wake_up_dur.sleep_dur(),
        };

        Ok(val)
    }
    // pub fn npy_floatbits_to_halfbits(&mut self, f: u32) -> u16 {
    //     let f_exp: u32;
    //     let f_sig: u32;
    //     let h_sgn: u16;
    //     let h_exp: u16;
    //     let h_sig: u16;

    //     h_sgn = ((f & 0x80000000) >> 16) as u16;
    //     f_exp = f & 0x7f800000;

    //     // Exponent overflow/NaN converts to signed inf/NaN
    //     if f_exp >= 0x47800000 {
    //         if f_exp == 0x7f800000 {
    //             // Inf or NaN
    //             f_sig = f & 0x007fffff;
    //             if f_sig != 0 {
    //                 // NaN - propagate the flag in the significand...
    //                 let mut ret = (0x7c00 + (f_sig >> 13)) as u16;
    //                 // ...but make sure it stays a NaN
    //                 if ret == 0x7c00 {
    //                     ret += 1;
    //                 }
    //                 return h_sgn + ret;
    //             } else {
    //                 // signed inf
    //                 return h_sgn + 0x7c00;
    //             }
    //         } else {
    //             // overflow to signed inf
    //             if NPY_HALF_GENERATE_OVERFLOW {
    //                 // npy_set_floatstatus_overflow(); // Implement this if needed
    //             }
    //             return h_sgn + 0x7c00;
    //         }
    //     }

    //     // Exponent underflow converts to a subnormal half or signed zero
    //     if f_exp <= 0x38000000 {
    //         // Signed zeros, subnormal floats, and floats with small exponents all convert to signed zero half-floats.
    //         if f_exp < 0x33000000 {
    //             if NPY_HALF_GENERATE_UNDERFLOW {
    //                 // If f != 0, it underflowed to 0
    //                 if (f & 0x7fffffff) != 0 {
    //                     // npy_set_floatstatus_underflow(); // Implement this if needed
    //                 }
    //             }
    //             return h_sgn;
    //         }
    //         // Make the subnormal significand
    //         let mut f_exp = f_exp >> 23;
    //         f_sig = 0x00800000 + (f & 0x007fffff);
    //         if NPY_HALF_GENERATE_UNDERFLOW {
    //             // If it's not exactly represented, it underflowed
    //             if (f_sig & ((1 << (126 - f_exp)) - 1)) != 0 {
    //                 // npy_set_floatstatus_underflow(); // Implement this if needed
    //             }
    //         }
    //         // Usually the significand is shifted by 13. For subnormals an additional shift needs to occur.
    //         f_sig >>= 113 - f_exp;
    //         // Handle rounding by adding 1 to the bit beyond half precision
    //         if NPY_HALF_ROUND_TIES_TO_EVEN {
    //             if ((f_sig & 0x00003fff) != 0x00001000) || (f & 0x000007ff) != 0 {
    //                 f_sig += 0x00001000;
    //             }
    //         } else {
    //             f_sig += 0x00001000;
    //         }
    //         h_sig = (f_sig >> 13) as u16;
    //         // If the rounding causes a bit to spill into h_exp, it will increment h_exp from zero to one and h_sig will be zero.
    //         return h_sgn + h_sig;
    //     }

    //     // Regular case with no overflow or underflow
    //     h_exp = ((f_exp - 0x38000000) >> 13) as u16;
    //     f_sig = f & 0x007fffff;
    //     // Handle rounding by adding 1 to the bit beyond half precision
    //     if NPY_HALF_ROUND_TIES_TO_EVEN {
    //         if (f_sig & 0x00003fff) != 0x00001000 {
    //             f_sig += 0x00001000;
    //         }
    //     } else {
    //         f_sig += 0x00001000;
    //     }
    //     h_sig = (f_sig >> 13) as u16;
    //     // If the rounding causes a bit to spill into h_exp, it will increment h_exp by one and h_sig will be zero.
    //     if NPY_HALF_GENERATE_OVERFLOW {
    //         let mut h_sig = h_sig + h_exp;
    //         if h_sig == 0x7c00 {
    //             // npy_set_floatstatus_overflow(); // Implement this if needed
    //         }
    //         return h_sgn + h_sig;
    //     } else {
    //         return h_sgn + h_exp + h_sig;
    //     }
    // }
    pub fn npy_float_to_half(&mut self, f: f32) -> u16 {
        //let fbits: u32 = f.to_bits();
        //self.npy_floatbits_to_halfbits(bits)
        let half_float = f16::from_f32(f);
        half_float.to_bits()
    }
}

pub fn from_sflp_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.061
}

pub fn from_fs2_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.061
}

pub fn from_fs4_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.122
}

pub fn from_fs8_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.244
}

pub fn from_fs16_to_mg(lsb: i16) -> f32 {
    (lsb as f32) * 0.488
}

pub fn from_fs125_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 4.375
}

pub fn from_fs250_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 8.750
}

pub fn from_fs500_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 17.50
}

pub fn from_fs1000_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 35.0
}

pub fn from_fs2000_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 70.0
}

pub fn from_fs4000_to_mdps(lsb: i16) -> f32 {
    (lsb as f32) * 140.0
}

pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    (lsb as f32) / 256.0 + 25.0
}

pub fn from_lsb_to_nsec(lsb: u32) -> f32 {
    (lsb as f32) * 21750.0
}

pub fn from_lsb_to_mv(lsb: i16) -> f32 {
    (lsb as f32) / 78.0
}

/*
 * Original conversion routines taken from: https://github.com/numpy/numpy
 *
 * Converts from half-precision (16-bit) float number to single precision (32-bit).
 *
 * uint32_t  static uint32_t ToFloatBits(uint16_t h);
 * Released under BSD-3-Clause License
 */
pub fn from_half_to_single_precision(h: u16) -> u32 {
    let mut h_exp = h & 0x7c00;
    let f_sgn: u32 = ((h as u32) & 0x8000) << 16;
    match h_exp {
        0x0000 => {
            // 0 or subnormal
            let mut h_sig = h & 0x03ff;
            // Signed zero
            if h_sig == 0 {
                return f_sgn;
            }
            // Subnormal
            h_sig <<= 1;
            while (h_sig & 0x0400) == 0 {
                h_sig <<= 1;
                h_exp += 1;
            }

            let f_exp = ((127 - 15 - h_exp) as u32) << 23;
            let f_sig = ((h_sig & 0x03ff) as u32) << 13;
            f_sgn + f_exp + f_sig
        }
        0x7c00 => {
            //inf or NaN
            // All-ones exponent and a copy of the significand
            f_sgn + 0x7f800000 + (((h & 0x03ff) as u32) << 13)
        }
        _ => {
            // normalized
            f_sgn + (((h & 0x7fff) as u32) << 13)
        }
    }
}

#[cfg(feature = "passthrough")]
pub struct Lsm6dsv16xMaster<B, T> {
    pub sensor: RefCell<Lsm6dsv16x<B, T>>,
}

#[cfg(feature = "passthrough")]
impl<B: BusOperation, T: DelayNs> Lsm6dsv16xMaster<B, T> {
    pub fn from_bus(bus: B, tim: T) -> Self {
        Self {
            sensor: RefCell::new(Lsm6dsv16x::from_bus(bus, tim)),
        }
    }

    pub fn borrow_mut(&self) -> RefMut<Lsm6dsv16x<B, T>> {
        self.sensor.borrow_mut()
    }

    /// Generates a wrapper for the sensor to enable its use as a passthrough
    /// from another sensor.
    ///
    /// The Sensor Hub may require this setup to redirect writes from the
    /// bus to the sensor that executes them as a passthrough.
    pub fn as_passthrough<'a>(
        &'a self,
        address: SevenBitAddress,
    ) -> Lsm6dsv16xPassthrough<'a, B, T> {
        Lsm6dsv16xPassthrough {
            sensor: &self.sensor,
            slave_address: address,
        }
    }
}

#[cfg(feature = "passthrough")]
/// Struct to handle Sensor to do the passthrough write and read for the
/// slave sensor.
///
/// Do not usit as it, call the as_passthrough function on the Lsm6dsv16x instance
pub struct Lsm6dsv16xPassthrough<'a, B, T> {
    sensor: &'a RefCell<Lsm6dsv16x<B, T>>,
    slave_address: SevenBitAddress,
}

#[cfg(feature = "passthrough")]
// LSM6DSV16X acts like a bus when used for the sensor hub.
impl<B, T> BusOperation for Lsm6dsv16xPassthrough<'_, B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    type Error = Error<B::Error>;

    fn read_bytes(&mut self, _rbuf: &mut [u8]) -> Result<(), Self::Error> {
        Err(Error::UnexpectedValue)
    }

    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        let mut master = self.sensor.borrow_mut();
        for i in 1_u8..(wbuf.len() as u8) {
            // Configure Sensor Hub to read data
            let sh_cfg_write = ShCfgWrite {
                slv0_add: self.slave_address,
                slv0_subadd: wbuf[0] + i - 1,
                slv0_data: wbuf[i as usize],
            };
            master.sh_cfg_write(sh_cfg_write)?;

            // Disable accelerometer
            master.xl_data_rate_set(Odr::Off)?;
            // Enable I2C Master
            master.sh_master_set(1)?;
            // Enable accelerometer to trigger Sensor Hub operation.
            master.xl_data_rate_set(Odr::_120hz)?;
            // Wait Sensor Hub operation flag set.
            master.acceleration_raw_get()?; // dummy read

            let mut drdy = 0;
            while drdy == 0 {
                master.tim.delay_ms(20);
                drdy = master.flag_data_ready_get()?.drdy_xl;
            }

            let mut end_op = 0;
            while end_op == 0 {
                master.tim.delay_ms(20);
                end_op = master.sh_status_get()?.sens_hub_endop();
            }

            // Disable I2C master and XL (triger).
            master.sh_master_set(0)?;
            master.xl_data_rate_set(Odr::Off)?;
        }

        Ok(())
    }

    fn write_byte_read_bytes(
        &mut self,
        wbuf: &[u8; 1],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        let mut master = self.sensor.borrow_mut();
        // Disable accelerometer
        master.xl_data_rate_set(Odr::Off)?;
        // Configure Sensor Hub to read
        let sh_cfg_read = ShCfgRead {
            slv_add: self.slave_address,
            slv_subadd: wbuf[0],
            slv_len: rbuf.len() as u8,
        };
        master.sh_slv_cfg_read(0, &sh_cfg_read)?; // dummy read
        master.sh_slave_connected_set(ShSlaveConnected::_01)?;
        // Enable I2C Master
        master.sh_master_set(1)?;
        // Enable accelerometer to trigger Sensor Hub operation.
        master.xl_data_rate_set(Odr::_120hz)?;
        // Wait Sensor Hub operation flag set
        master.acceleration_raw_get()?; // dummy read

        let mut drdy = 0;
        while drdy == 0 {
            master.tim.delay_ms(20);
            drdy = master.flag_data_ready_get()?.drdy_xl;
        }

        let mut end_op = 0;
        while end_op == 0 {
            //master.tim.delay_ms(20);
            end_op = master.sh_status_get()?.sens_hub_endop();
        }

        // Disable I2C master and XL(trigger)
        master.sh_master_set(0)?;
        master.xl_data_rate_set(Odr::Off)?;

        // Read SensorHub registers
        master.sh_read_data_raw_get(rbuf)
    }
}

#[derive(Clone, Copy, Default)]
pub struct AllSources {
    pub drdy_xl: u8,
    pub drdy_gy: u8,
    pub drdy_temp: u8,
    pub drdy_ah_qvar: u8,
    pub drdy_eis: u8,
    pub drdy_ois: u8,
    pub gy_settling: u8,
    pub timestamp: u8,
    pub free_fall: u8,
    pub wake_up: u8,
    pub wake_up_z: u8,
    pub wake_up_y: u8,
    pub wake_up_x: u8,
    pub single_tap: u8,
    pub double_tap: u8,
    pub tap_z: u8,
    pub tap_y: u8,
    pub tap_x: u8,
    pub tap_sign: u8,
    pub six_d: u8,
    pub six_d_xl: u8,
    pub six_d_xh: u8,
    pub six_d_yl: u8,
    pub six_d_yh: u8,
    pub six_d_zl: u8,
    pub six_d_zh: u8,
    pub sleep_change: u8,
    pub sleep_state: u8,
    pub step_detector: u8,
    pub step_count_inc: u8,
    pub step_count_overflow: u8,
    pub step_on_delta_time: u8,
    pub emb_func_stand_by: u8,
    pub emb_func_time_exceed: u8,
    pub tilt: u8,
    pub sig_mot: u8,
    pub fsm_lc: u8,
    pub fsm1: u8,
    pub fsm2: u8,
    pub fsm3: u8,
    pub fsm4: u8,
    pub fsm5: u8,
    pub fsm6: u8,
    pub fsm7: u8,
    pub fsm8: u8,
    pub mlc1: u8,
    pub mlc2: u8,
    pub mlc3: u8,
    pub mlc4: u8,
    pub sh_endop: u8,
    pub sh_slave0_nack: u8,
    pub sh_slave1_nack: u8,
    pub sh_slave2_nack: u8,
    pub sh_slave3_nack: u8,
    pub sh_wr_once: u8,
    pub fifo_bdr: u8,
    pub fifo_full: u8,
    pub fifo_ovr: u8,
    pub fifo_th: u8,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    I2cAddL = 0x6A,
    I2cAddH = 0x6B,
}

pub const ID: u8 = 0x70;

// const NPY_HALF_GENERATE_OVERFLOW: bool = false; // do not trigger FP overflow
// const NPY_HALF_GENERATE_UNDERFLOW: bool = false; // do not trigger FP underflow
// const NPY_HALF_ROUND_TIES_TO_EVEN: bool = true;
