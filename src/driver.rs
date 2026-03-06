use super::{
    BusOperation, DelayNs, EmbAdvFunctions, I2c, MemBankFunctions, RegisterOperation,
    SensorOperation, SevenBitAddress, SpiDevice, bisync, i2c, prelude::*, register::BankState,
    register::*, spi,
};

use core::fmt::Debug;
use core::marker::PhantomData;
use half::f16;

/// Driver for the LSM6DSV16X sensor.
///
/// The struct takes a bus and a timer hardware object to write to the
/// registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
#[bisync]
pub struct Lsm6dsv16x<B: BusOperation, T: DelayNs, S: BankState> {
    pub bus: B,
    pub tim: T,
    _state: PhantomData<S>,
}

/// Driver errors.
#[derive(Debug)]
#[bisync]
pub enum Error<B> {
    /// Incapsulate bus errors
    Bus(B),
    /// Unexpected value read from a register
    UnexpectedValue,
    FailedToReadMemBank,
    FailedToSetMemBank(MemBank),
}

#[bisync]
impl<B, T, S> Lsm6dsv16x<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: BankState,
{
    /// Create a sensor instance from a generic bus.
    ///
    /// The Bus must implement the `BusOperation` trait
    pub fn from_bus(bus: B, tim: T) -> Self {
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Lsm6dsv16x<i2c::I2cBus<P>, T, MainBank>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Lsm6dsv16x<spi::SpiBus<P>, T, MainBank>
where
    P: SpiDevice,
    T: DelayNs,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<B, T, S> MemBankFunctions<MemBank> for Lsm6dsv16x<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: BankState,
{
    type Error = Error<B::Error>;

    /// Change memory bank.
    async fn mem_bank_set(&mut self, val: MemBank) -> Result<(), Self::Error> {
        let mut func_cfg_access = FuncCfgAccess::read(self)
            .await
            .map_err(|_| Error::FailedToReadMemBank)?;

        func_cfg_access.set_shub_reg_access(((val as u8) & 0x02) >> 1);
        func_cfg_access.set_emb_func_reg_access(val as u8 & 0x01);
        func_cfg_access
            .write(self)
            .await
            .map_err(|_| Error::FailedToSetMemBank(val))
    }

    /// Get the memory bank actually set.
    async fn mem_bank_get(&mut self) -> Result<MemBank, Self::Error> {
        let func_cfg_access = FuncCfgAccess::read(self)
            .await
            .map_err(|_| Error::FailedToReadMemBank)?;
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

#[bisync]
impl<B, T> EmbAdvFunctions for Lsm6dsv16x<B, T, MainBank>
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
    async fn ln_pg_write(&mut self, address: u16, buf: &[u8], len: u8) -> Result<(), Self::Error> {
        let mut msb = ((address >> 8) & 0x0F) as u8;
        let mut lsb = (address & 0xFF) as u8;

        self.operate_over_embed(async |state| {
            // Set page write
            let mut page_rw = PageRw::read(state).await?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(1);
            page_rw.write(state).await?;

            // Select page
            let mut page_sel = PageSel::read(state).await?;
            page_sel.set_page_sel(msb);
            page_sel.write(state).await?;

            // Set page address
            let mut page_address = PageAddress::from_bits(0);
            page_address.set_page_addr(lsb);
            page_address.write(state).await?;

            for i in 0..len {
                state
                    .write_to_register(EmbReg::PageValue as u8, &buf[i as usize..(i as usize + 1)])
                    .await?;

                lsb = lsb.wrapping_add(1);
                // Check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(state).await?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(state).await?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(state).await?;
            page_sel.set_page_sel(0);
            page_sel.write(state).await?;

            // Unset page write
            page_rw = PageRw::read(state).await?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(0);
            page_rw.write(state).await
        })
        .await
    }

    /// Read buffer in a page.
    ///
    /// # Arguments
    ///
    /// * `address`: The address to read from.
    /// * `buf`: Write buffer in a page.
    /// * `len`: Length of the buffer.
    async fn ln_pg_read(
        &mut self,
        address: u16,
        buf: &mut [u8],
        len: u8,
    ) -> Result<(), Self::Error> {
        let mut msb = ((address >> 8) & 0x0F) as u8;
        let mut lsb = (address & 0xFF) as u8;

        self.operate_over_embed(async |state| {
            // Set page read
            let mut page_rw = PageRw::read(state).await?;
            page_rw.set_page_read(1);
            page_rw.set_page_write(0);
            page_rw.write(state).await?;

            // Select page
            let mut page_sel = PageSel::read(state).await?;
            page_sel.set_page_sel(msb);
            page_sel.write(state).await?;

            // Set page address
            let mut page_address = PageAddress::from_bits(0);
            page_address.set_page_addr(lsb);
            page_address.write(state).await?;

            for i in 0..len {
                state
                    .read_from_register(
                        EmbReg::PageValue as u8,
                        &mut buf[i as usize..(i as usize + 1)],
                    )
                    .await?;

                lsb = lsb.wrapping_add(1);
                // Check if page wrap
                if lsb == 0x00 {
                    msb += 1;
                    page_sel = PageSel::read(state).await?;
                    page_sel.set_page_sel(msb);
                    page_sel.write(state).await?;
                }
            }

            // Reset page selection
            page_sel = PageSel::read(state).await?;
            page_sel.set_page_sel(0);
            page_sel.write(state).await?;

            // Unset page read
            page_rw = PageRw::read(state).await?;
            page_rw.set_page_read(0);
            page_rw.set_page_write(0);
            page_rw.write(state).await
        })
        .await
    }
}

#[bisync]
impl<B: BusOperation, T: DelayNs, S: BankState> SensorOperation for Lsm6dsv16x<B, T, S> {
    type Error = Error<B::Error>;
    async fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .write_to_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }

    async fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .read_from_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }
}

#[bisync]
impl<B: BusOperation, T: DelayNs> Lsm6dsv16x<B, T, MainBank> {
    /// Enables accelerometer user offset correction block.
    ///
    /// It is valid for the low-pass path.
    pub async fn xl_offset_on_out_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self).await?;
        ctrl9.set_usr_off_on_out(val);
        ctrl9.write(self).await?;
        Ok(())
    }
    /// Get accelerometer user offset correction block (enable/disable).
    ///
    /// It is valid for the low-pass path.
    pub async fn xl_offset_on_out_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).await.map(|reg| reg.usr_off_on_out())?;

        Ok(val)
    }
    /// Set the Accelerometer user offset correction values in mg.
    pub async fn xl_offset_mg_set(&mut self, val: XlOffsetMg) -> Result<(), Error<B::Error>> {
        let mut z_ofs_usr = ZOfsUsr::read(self).await?;
        let mut y_ofs_usr = YOfsUsr::read(self).await?;
        let mut x_ofs_usr = XOfsUsr::read(self).await?;

        let mut ctrl9 = Ctrl9::read(self).await?;

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

        z_ofs_usr.write(self).await?;
        y_ofs_usr.write(self).await?;
        x_ofs_usr.write(self).await?;
        ctrl9.write(self).await
    }
    /// Get the Accelerometer user offset correction values in mg.
    pub async fn xl_offset_mg_get(&mut self) -> Result<XlOffsetMg, Error<B::Error>> {
        let ctrl9 = Ctrl9::read(self).await?;
        let z_ofs_usr = ZOfsUsr::read(self).await?;
        let y_ofs_usr = YOfsUsr::read(self).await?;
        let x_ofs_usr = XOfsUsr::read(self).await?;

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
    pub async fn reset_set(&mut self, val: Reset) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self).await?;
        let mut func_cfg_access = FuncCfgAccess::read(self).await?;

        ctrl3.set_boot(if val == Reset::RestoreCalParam { 1 } else { 0 });
        ctrl3.set_sw_reset(if val == Reset::RestoreCtrlRegs { 1 } else { 0 });
        func_cfg_access.set_sw_por(if val == Reset::GlobalRst { 1 } else { 0 });

        ctrl3.write(self).await?;
        func_cfg_access.write(self).await
    }
    /// Get reset state of the device.
    pub async fn reset_get(&mut self) -> Result<Reset, Error<B::Error>> {
        let ctrl3 = Ctrl3::read(self).await?;
        let func_cfg_access = FuncCfgAccess::read(self).await?;

        let value: u8 = (ctrl3.sw_reset() << 2) + (ctrl3.boot() << 1) + func_cfg_access.sw_por();
        let val = Reset::try_from(value).unwrap_or(Reset::GlobalRst);

        Ok(val)
    }

    /// Get Device ID.
    ///
    /// This function works also for OIS
    /// (WHO_AM_I and SPI2_WHO_AM_I have same address).
    pub async fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).await.map(|reg| reg.into())
    }
    /// Set Accelerometer output data rate (ODR).
    pub async fn xl_data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut ctrl1 = Ctrl1::read(self).await?;

        ctrl1.set_odr_xl(val as u8 & 0x0F);
        ctrl1.write(self).await?;

        let sel = (val as u8 >> 4) & 0x0F;
        if sel != 0 {
            let mut haodr = HaodrCfg::read(self).await?;

            haodr.set_haodr_sel(sel);
            haodr.write(self).await?;
        }

        Ok(())
    }
    /// Get accelerometer output data rate (ODR).
    pub async fn xl_data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let ctrl1 = Ctrl1::read(self).await?;
        let haodr = HaodrCfg::read(self).await?;
        let sel = haodr.haodr_sel() << 4;
        let odr = sel | ctrl1.odr_xl();

        Ok(Odr::try_from(odr).unwrap_or_default())
    }
    /// Set accelerometer operating mode.
    pub async fn xl_mode_set(&mut self, val: XlMode) -> Result<(), Error<B::Error>> {
        let mut ctrl1 = Ctrl1::read(self).await?;
        ctrl1.set_op_mode_xl((val as u8) & 0x07);
        ctrl1.write(self).await
    }
    /// Get accelerometer operating mode.
    pub async fn xl_mode_get(&mut self) -> Result<XlMode, Error<B::Error>> {
        let ctrl1 = Ctrl1::read(self).await?;

        let val = XlMode::try_from(ctrl1.op_mode_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Set gyroscope output data rate (ODR).
    pub async fn gy_data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        let mut ctrl2 = Ctrl2::read(self).await?;

        ctrl2.set_odr_g((val as u8) & 0x0F);
        ctrl2.write(self).await?;

        let sel = ((val as u8) >> 4) & 0x0F;
        if sel != 0 {
            let mut haodr = HaodrCfg::read(self).await?;
            haodr.set_haodr_sel(sel);
            haodr.write(self).await?;
        }

        Ok(())
    }
    /// Get gyroscope output data rate (ODR).
    pub async fn gy_data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let ctrl2 = Ctrl2::read(self).await?;
        let haodr = HaodrCfg::read(self).await?;
        let sel = haodr.haodr_sel();
        let odr = sel | ctrl2.odr_g();

        Ok(Odr::try_from(odr).unwrap_or_default())
    }
    /// Set gyroscope operating mode.
    pub async fn gy_mode_set(&mut self, val: GyMode) -> Result<(), Error<B::Error>> {
        let mut ctrl2 = Ctrl2::read(self).await?;
        ctrl2.set_op_mode_g(val as u8 & 0x07);
        ctrl2.write(self).await
    }
    /// Get gyroscope operating mode.
    pub async fn gy_mode_get(&mut self) -> Result<GyMode, Error<B::Error>> {
        let ctrl2 = Ctrl2::read(self).await?;

        let val = GyMode::try_from(ctrl2.op_mode_g()).unwrap_or_default();
        Ok(val)
    }
    /// Enable/Disable the auto increment setting.
    ///
    /// If val == 1 it Enable automatic increment of the register address during
    /// multiple-byte access with a serial interface; enabled by default.
    pub async fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self).await?;
        ctrl3.set_if_inc(val);
        ctrl3.write(self).await
    }
    /// Get the actual auto increment setting
    ///
    /// Register address automatically incremented during a multiple byte access
    /// with a serial interface (enable by default).
    pub async fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl3::read(self).await.map(|reg| reg.if_inc())?;

        Ok(val)
    }
    /// Enable/Disable Block Data Update (BDU)
    ///
    /// If active the output registers are not updated until LSB and MSB have been read.
    pub async fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl3 = Ctrl3::read(self).await?;
        ctrl3.set_bdu(val);
        ctrl3.write(self).await
    }
    /// Get actual settings of Block Data Update (BDU)
    pub async fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl3::read(self).await.map(|reg| reg.bdu())?;

        Ok(val)
    }
    /// Configure ODR trigger.
    ///
    /// Number of data generated in reference period when ODR-triggered mode is set.
    /// Allowed values: 0 (default) or 4 to 255.
    pub async fn odr_trig_cfg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        if (1..=3).contains(&val) {
            return Err(Error::UnexpectedValue);
        }

        let mut odr_trig = OdrTrigCfg::read(self).await?;

        odr_trig.set_odr_trig_nodr(val);
        odr_trig.write(self).await
    }
    /// Get the actual ODR trigger.
    ///
    /// Number of data generated in reference period when ODR-triggered mode is set.
    pub async fn odr_trig_cfg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let odr_trig_nodr: u8 = OdrTrigCfg::read(self)
            .await
            .map(|reg| reg.odr_trig_nodr())?;

        Ok(odr_trig_nodr)
    }
    /// Switch between pulsed and latched mode.
    ///
    /// Pulsed data-ready mode with ~75 us.
    pub async fn data_ready_mode_set(&mut self, val: DataReadyMode) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = Ctrl4::read(self).await?;
        ctrl4.set_drdy_pulsed((val as u8) & 0x1);
        ctrl4.write(self).await
    }
    /// Get actual pulsed data-ready mode
    pub async fn data_ready_mode_get(&mut self) -> Result<DataReadyMode, Error<B::Error>> {
        let ctrl4 = Ctrl4::read(self).await?;
        let val = DataReadyMode::try_from(ctrl4.drdy_pulsed()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/disable interrupt and switch between latched/pulsed interrupts
    pub async fn interrupt_enable_set(
        &mut self,
        val: InterruptMode,
    ) -> Result<(), Error<B::Error>> {
        let mut func = FunctionsEnable::read(self).await?;
        func.set_interrupts_enable(val.enable);
        func.write(self).await?;

        let mut cfg = TapCfg0::read(self).await?;
        cfg.set_lir(val.lir);
        cfg.write(self).await
    }
    /// Get the interrupt Mode
    ///
    /// Enable/disabled and latched/pulsed information.
    pub async fn interrupt_enable_get(&mut self) -> Result<InterruptMode, Error<B::Error>> {
        let func = FunctionsEnable::read(self).await?;
        let cfg = TapCfg0::read(self).await?;

        let enable = func.interrupts_enable();
        let lir = cfg.lir();

        Ok(InterruptMode { enable, lir })
    }
    /// Set the Gyroscope full-scale.
    pub async fn gy_full_scale_set(&mut self, val: GyFullScale) -> Result<(), Error<B::Error>> {
        let mut ctrl6 = Ctrl6::read(self).await?;

        ctrl6.set_fs_g(val as u8 & 0xf);
        ctrl6.write(self).await
    }
    /// Get the actual Gyroscope full-scale.
    pub async fn gy_full_scale_get(&mut self) -> Result<GyFullScale, Error<B::Error>> {
        let ctrl6 = Ctrl6::read(self).await?;

        let val = GyFullScale::try_from(ctrl6.fs_g()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Accelerometer full-scale.
    pub async fn xl_full_scale_set(&mut self, val: XlFullScale) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self).await?;
        ctrl8.set_fs_xl(val as u8 & 0x3);
        ctrl8.write(self).await
    }
    /// Get the Accelerometer full-scale.
    pub async fn xl_full_scale_get(&mut self) -> Result<XlFullScale, Error<B::Error>> {
        let ctrl8 = Ctrl8::read(self).await?;

        let val = XlFullScale::try_from(ctrl8.fs_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables the accelerometer Dual channel mode:
    ///
    /// data with selected full scale and data with maximum full scale are sent
    /// simultaneously to two different set of output registers.
    pub async fn xl_dual_channel_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self).await?;
        ctrl8.set_xl_dualc_en(val);
        ctrl8.write(self).await
    }
    /// Get the accelerometer Dual channel mode.
    ///
    /// Data with selected full scale and data with maximum full scale are sent
    /// simultaneously to two different set of output registers.
    pub async fn xl_dual_channel_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl8::read(self).await.map(|reg| reg.xl_dualc_en())?;

        Ok(val)
    }
    /// Set the Accelerometer self-test.
    pub async fn xl_self_test_set(&mut self, val: XlSelfTest) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self).await?;
        ctrl10.set_st_xl(val as u8 & 0x3);
        ctrl10.write(self).await
    }
    /// Get the actual Accelerometer self-test.
    pub async fn xl_self_test_get(&mut self) -> Result<XlSelfTest, Error<B::Error>> {
        let ctrl10 = Ctrl10::read(self).await?;

        let val = XlSelfTest::try_from(ctrl10.st_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Gyroscope self-test.
    pub async fn gy_self_test_set(&mut self, val: GySelfTest) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self).await?;
        ctrl10.set_st_g((val as u8) & 0x3);
        ctrl10.write(self).await
    }
    /// Get the actual Gyroscope self-test selection.
    pub async fn gy_self_test_get(&mut self) -> Result<GySelfTest, Error<B::Error>> {
        let ctrl10 = Ctrl10::read(self).await?;

        let val = GySelfTest::try_from(ctrl10.st_g()).unwrap_or_default();

        Ok(val)
    }
    /// Set the SPI2 Accelerometer self-test.
    pub async fn ois_xl_self_test_set(
        &mut self,
        val: OisXlSelfTest,
    ) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self).await?;
        spi2_int_ois.set_st_xl_ois((val as u8) & 0x3);
        spi2_int_ois.write(self).await
    }
    /// Get the actual SPI2 Accelerometer self-test.
    pub async fn ois_xl_self_test_get(&mut self) -> Result<OisXlSelfTest, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self).await?;
        let val = OisXlSelfTest::try_from(spi2_int_ois.st_xl_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Set SPI2 Accelerometer self-test.
    pub async fn ois_gy_self_test_set(
        &mut self,
        val: OisGySelfTest,
    ) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self).await?;

        spi2_int_ois.set_st_g_ois((val as u8) & 0x3);
        spi2_int_ois.set_st_ois_clampdis(((val as u8) & 0x04) >> 2);

        spi2_int_ois.write(self).await
    }
    /// Get the actual SPI2 Accelerometer self-test.
    pub async fn ois_gy_self_test_get(&mut self) -> Result<OisGySelfTest, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self).await?;

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
    pub async fn pin_int1_route_set(&mut self, val: &PinIntRoute) -> Result<(), Error<B::Error>> {
        // Check if drdy_temp is set to 1, which is not available on INT1
        if val.drdy_temp == 1 {
            return Err(Error::UnexpectedValue);
        }

        // Read INT1_CTRL register
        let mut int1_ctrl = Int1Ctrl::read(self).await?;

        // Set fields from `val` to `int1_ctrl`
        int1_ctrl.set_int1_drdy_xl(val.drdy_xl);
        int1_ctrl.set_int1_drdy_g(val.drdy_g);
        int1_ctrl.set_int1_fifo_th(val.fifo_th);
        int1_ctrl.set_int1_fifo_ovr(val.fifo_ovr);
        int1_ctrl.set_int1_fifo_full(val.fifo_full);
        int1_ctrl.set_int1_cnt_bdr(val.cnt_bdr);

        // Write back to INT1_CTRL register
        int1_ctrl.write(self).await?;

        // Read MD1_CFG register
        let mut md1_cfg = Md1Cfg::read(self).await?;

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
        md1_cfg.write(self).await
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
    pub async fn pin_int1_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        // Reading INT1 Control Register
        let int1_ctrl = Int1Ctrl::read(self).await?;

        // Reading MD1 Configuration Register
        let md1_cfg = Md1Cfg::read(self).await?;

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
    pub async fn pin_int2_route_set(&mut self, val: &PinIntRoute) -> Result<(), Error<B::Error>> {
        // Reading INT2 Control Register
        let mut int2_ctrl = Int2Ctrl::read(self).await?;

        int2_ctrl.set_int2_drdy_xl(val.drdy_xl);
        int2_ctrl.set_int2_drdy_g(val.drdy_g);
        int2_ctrl.set_int2_fifo_th(val.fifo_th);
        int2_ctrl.set_int2_fifo_ovr(val.fifo_ovr);
        int2_ctrl.set_int2_fifo_full(val.fifo_full);
        int2_ctrl.set_int2_cnt_bdr(val.cnt_bdr);
        int2_ctrl.set_int2_drdy_g_eis(val.drdy_g_eis);
        int2_ctrl.set_int2_emb_func_endop(val.emb_func_endop);

        int2_ctrl.write(self).await?;

        // Reading CTRL4 Register
        let mut ctrl4 = Ctrl4::read(self).await?;

        ctrl4.set_int2_drdy_temp(val.drdy_temp);
        ctrl4.write(self).await?;

        // Reading CTRL7 Register
        let mut ctrl7 = Ctrl7::read(self).await?;

        ctrl7.set_int2_drdy_ah_qvar(val.drdy_ah_qvar);
        ctrl7.write(self).await?;
        // Reading MD2 Configuration Register
        let mut md2_cfg = Md2Cfg::read(self).await?;

        md2_cfg.set_int2_timestamp(val.timestamp);
        md2_cfg.set_int2_emb_func(val.emb_func);
        md2_cfg.set_int2_6d(val.sixd);
        md2_cfg.set_int2_single_tap(val.single_tap);
        md2_cfg.set_int2_double_tap(val.double_tap);
        md2_cfg.set_int2_wu(val.wakeup);
        md2_cfg.set_int2_ff(val.freefall);
        md2_cfg.set_int2_sleep_change(val.sleep_change);

        md2_cfg.write(self).await
    }
    /// Report the signals that are routed on int2 pad.
    /// Values NOT returned are:
    ///     - shub
    pub async fn pin_int2_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        // Read from INT2_CTRL register
        let int2_ctrl = Int2Ctrl::read(self).await?;
        // Read from CTRL4 register
        let ctrl4 = Ctrl4::read(self).await?;
        // Read from CTRL7 register
        let ctrl7 = Ctrl7::read(self).await?;
        // Read from MD2_CFG register
        let md2_cfg = Md2Cfg::read(self).await?;

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
    pub async fn emb_pin_int1_route_get(&mut self) -> Result<EmbPinIntRoute, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let emb_func_int1 = EmbFuncInt1::read(state).await?;

            let val = EmbPinIntRoute {
                tilt: emb_func_int1.int1_tilt(),
                sig_mot: emb_func_int1.int1_sig_mot(),
                step_det: emb_func_int1.int1_step_detector(),
                fsm_lc: emb_func_int1.int1_fsm_lc(),
            };

            Ok(val)
        })
        .await
    }

    /// Routes embedded func interrupt signals on INT 1 pin.
    pub async fn emb_pin_int1_route_set(
        &mut self,
        val: EmbPinIntRoute,
    ) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_int1 = EmbFuncInt1::read(state).await?;

            emb_func_int1.set_int1_tilt(val.tilt);
            emb_func_int1.set_int1_sig_mot(val.sig_mot);
            emb_func_int1.set_int1_step_detector(val.step_det);
            emb_func_int1.set_int1_fsm_lc(val.fsm_lc);

            emb_func_int1.write(state).await
        })
        .await?;

        let mut md1_cfg = Md1Cfg::read(self).await?;
        md1_cfg.set_int1_emb_func(1);
        md1_cfg.write(self).await
    }

    /// Get routed embedded func interrupt signals on INT 2 pin.
    pub async fn emb_pin_int2_route_get(&mut self) -> Result<EmbPinIntRoute, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let emb_func_int2 = EmbFuncInt2::read(state).await?;

            let val = EmbPinIntRoute {
                tilt: emb_func_int2.int2_tilt(),
                sig_mot: emb_func_int2.int2_sig_mot(),
                step_det: emb_func_int2.int2_step_detector(),
                fsm_lc: emb_func_int2.int2_fsm_lc(),
            };

            Ok(val)
        })
        .await
    }

    /// Routes embedded func interrupt signals on INT 2 pin.
    pub async fn emb_pin_int2_route_set(
        &mut self,
        val: EmbPinIntRoute,
    ) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_int2 = EmbFuncInt2::read(state).await?;

            emb_func_int2.set_int2_tilt(val.tilt);
            emb_func_int2.set_int2_sig_mot(val.sig_mot);
            emb_func_int2.set_int2_step_detector(val.step_det);
            emb_func_int2.set_int2_fsm_lc(val.fsm_lc);

            emb_func_int2.write(state).await
        })
        .await?;

        let mut md2_cfg = Md2Cfg::read(self).await?;
        md2_cfg.set_int2_emb_func(1);
        md2_cfg.write(self).await
    }

    /// Get interrupt configuration mode.
    pub async fn embedded_int_cfg_get(&mut self) -> Result<EmbeddedIntConf, Error<B::Error>> {
        let mut val = EmbeddedIntConf::Pulsed;

        self.operate_over_embed(async |state| {
            let page_rw = PageRw::read(state).await?;
            if page_rw.emb_func_lir() == 1 {
                val = EmbeddedIntConf::Latched;
            }

            Ok(val)
        })
        .await
    }

    /// Set interrupt configuration mode.
    pub async fn embedded_int_cfg_set(
        &mut self,
        val: EmbeddedIntConf,
    ) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut page_rw = PageRw::read(state).await?;
            match val {
                EmbeddedIntConf::Latched => {
                    page_rw.set_emb_func_lir(1);
                }
                EmbeddedIntConf::Pulsed => {
                    page_rw.set_emb_func_lir(0);
                }
            };
            page_rw.write(state).await
        })
        .await
    }

    /// Get the status of embedded functions.
    pub async fn embedded_status_get(&mut self) -> Result<EmbeddedStatus, Error<B::Error>> {
        let status = EmbFuncStatusMainPage::read(self).await?;
        self.operate_over_embed(async |state| {
            let src = EmbFuncSrc::read(state).await?;

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
        .await
    }

    /// Get the status of all the interrupt sources.
    pub async fn all_sources_get(&mut self) -> Result<AllSources, Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self).await?;
        functions_enable.set_dis_rst_lir_all_int(1);
        functions_enable.write(self).await?;

        let fifo_status = FifoStatusReg::read(self).await?;
        let all_int_src = AllIntSrc::read(self).await?;
        let status_reg = StatusReg::read(self).await?;

        functions_enable = FunctionsEnable::read(self).await?;
        functions_enable.set_dis_rst_lir_all_int(0);
        functions_enable.write(self).await?;

        let status_reg_ois = UiStatusRegOis::read(self).await?;
        let wake_up_src = WakeUpSrc::read(self).await?;
        let tap_src = TapSrc::read(self).await?;
        let d6d_src = D6dSrc::read(self).await?;

        let emb_func_status_mainpage = EmbFuncStatusMainPage::read(self).await?;
        let fsm_status_mainpage = FsmStatusMainPage::read(self).await?;
        let mlc_status_mainpage = MlcStatusMainPage::read(self).await?;

        // Embedded function
        let (emb_func_exec_status, emb_func_src) = self
            .operate_over_embed(async |state| {
                let emb_func_exec_status = EmbFuncExecStatus::read(state).await?;
                let emb_func_src = EmbFuncSrc::read(state).await?;
                Ok((emb_func_exec_status, emb_func_src))
            })
            .await?;

        let status_shub = StatusMasterMainPage::read(self).await?;

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
    pub async fn flag_data_ready_get(&mut self) -> Result<DataReady, Error<B::Error>> {
        let status_reg = StatusReg::read(self).await?;

        let data_ready = DataReady {
            drdy_xl: status_reg.xlda(),
            drdy_gy: status_reg.gda(),
            drdy_temp: status_reg.tda(),
        };

        Ok(data_ready)
    }
    /// Set Mask status bit reset
    pub async fn int_ack_mask_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| IntAckMask::from_bits(val).write(state).await)
            .await
    }
    /// Get the actual Mask status bit reset
    pub async fn int_ack_mask_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(IntAckMask::read)
            .await
            .map(|reg| reg.into_bits())
    }
    /// Get the Temperature raw data
    pub async fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        OutTemp::read(self).await.map(|reg| reg.0)
    }

    /// Get the Angular rate raw data.
    pub async fn angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZG::read(self).await?;

        Ok([val.x, val.y, val.z])
    }
    /// Get the OIS Angular rate raw data.
    ///
    /// Data comes through SPI2
    pub async fn ois_angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = Spi2OutXYZGOis::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// Get angular rate raw data for OIS gyro or the EIS gyro channel.
    pub async fn ois_eis_angular_rate_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = UiOutXYZGOisEis::read(self).await?;

        Ok([val.x, val.y, val.z])
    }

    /// Get the Linear acceleration raw data.
    pub async fn acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = OutXYZA::read(self).await?;

        Ok([val.x, val.y, val.z])
    }
    /// Get the Linear acceleration sensor for Dual channel mode.
    pub async fn dual_acceleration_raw_get(&mut self) -> Result<[i16; 3], Error<B::Error>> {
        let val = UiOutXYZAOisDualc::read(self).await?;

        Ok([val.x, val.y, val.z])
    }
    /// Get ah_qvar raw data.
    ///
    /// When the analog hub or Qvar is enabled, the output is
    /// the analog hub or Qvar sensor data.
    pub async fn ah_qvar_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        AhQvarOut::read(self).await.map(|reg| reg.0)
    }
    /// Get difference in percentage of the effective ODR
    /// (and timestamp rate) with respect to the typical.
    ///
    /// Step: 0.13%. 8-bit format, 2's complement.
    pub async fn odr_cal_reg_get(&mut self) -> Result<i8, Error<B::Error>> {
        let val: i8 = InternalFreq::read(self)
            .await
            .map(|reg| reg.freq_fine() as i8)?;

        Ok(val)
    }

    /// Enable debug mode for embedded functions.
    pub async fn emb_function_dbg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl10 = Ctrl10::read(self).await?;
        ctrl10.set_emb_func_debug(val);
        ctrl10.write(self).await
    }
    /// Get if debug mode for embedded functions is enabled
    pub async fn emb_function_dbg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl10::read(self).await.map(|reg| reg.emb_func_debug())?;

        Ok(val)
    }
    /// It changes the polarity of INT2 pin input trigger for data enable (DEN) or embedded functions.
    pub async fn den_polarity_set(&mut self, val: DenPolarity) -> Result<(), Error<B::Error>> {
        let mut ctrl4 = Ctrl4::read(self).await?;
        ctrl4.set_int2_in_lh((val as u8) & 0x1);
        ctrl4.write(self).await
    }
    /// Get the polarity of INT2 pin input trigger for data enable (DEN) or embedded functions.
    pub async fn den_polarity_get(&mut self) -> Result<DenPolarity, Error<B::Error>> {
        let ctrl4 = Ctrl4::read(self).await?;

        let val = DenPolarity::try_from(ctrl4.int2_in_lh()).unwrap_or_default();

        Ok(val)
    }
    /// Set Data ENable (DEN) configuration.
    pub async fn den_conf_set(&mut self, val: DenConf) -> Result<(), Error<B::Error>> {
        let mut den = Den::read(self).await?;

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

        den.write(self).await
    }
    /// Get Data ENable (DEN) configuration.
    pub async fn den_conf_get(&mut self) -> Result<DenConf, Error<B::Error>> {
        let den = Den::read(self).await?;

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
    pub async fn eis_gy_full_scale_set(
        &mut self,
        val: EisGyFullScale,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self).await?;
        ctrl_eis.set_fs_g_eis(val as u8 & 0x7);
        ctrl_eis.write(self).await
    }
    /// Get the actual Gyroscope full-scale selection for EIS channel.
    ///
    /// WARNING: 4000dps will be available only if also User Interface chain is set to 4000dps
    pub async fn eis_gy_full_scale_get(&mut self) -> Result<EisGyFullScale, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self).await?;
        let val = EisGyFullScale::try_from(ctrl_eis.fs_g_eis()).unwrap_or_default();

        Ok(val)
    }
    /// Enables routing of gyroscope EIS outputs on SPI2 (OIS interface).
    ///
    /// The gyroscope data on SPI2 (OIS interface) cannot be read from User Interface (UI).
    pub async fn eis_gy_on_spi2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self).await?;
        ctrl_eis.set_g_eis_on_g_ois_out_reg(val);
        ctrl_eis.write(self).await
    }
    /// Enables routing of gyroscope EIS outputs on SPI2 (OIS interface).
    ///
    /// The gyroscope data on SPI2 (OIS interface) cannot be read from User Interface (UI).
    pub async fn eis_gy_on_spi2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlEis::read(self)
            .await
            .map(|reg| reg.g_eis_on_g_ois_out_reg())?;

        Ok(val)
    }
    /// Enables and selects the ODR of the gyroscope EIS channel.
    pub async fn gy_eis_data_rate_set(
        &mut self,
        val: GyEisDataRate,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self).await?;
        ctrl_eis.set_odr_g_eis((val as u8) & 0x03);
        ctrl_eis.write(self).await
    }
    /// Get the actual ODR of the gyroscope EIS channel.
    pub async fn gy_eis_data_rate_get(&mut self) -> Result<GyEisDataRate, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self).await?;

        let val = GyEisDataRate::try_from(ctrl_eis.odr_g_eis()).unwrap_or_default();

        Ok(val)
    }
    /// Set FIFO watermark threshold
    ///
    /// 1 LSb = TAG (1 Byte) + 1 sensor (6 Bytes) written in FIFO.
    pub async fn fifo_watermark_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl1 = FifoCtrl1::read(self).await?;
        fifo_ctrl1.set_wtm(val);
        fifo_ctrl1.write(self).await
    }
    /// Get the actual FIFO watermark threshold
    ///
    /// 1 LSb = TAG (1 Byte) + 1 sensor (6 Bytes) written in FIFO
    pub async fn fifo_watermark_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl1::read(self).await.map(|reg| reg.wtm())?;

        Ok(val)
    }
    /// Enables FSM-triggered batching in FIFO of accelerometer channel 2, WHEN dual channel mode
    /// is enabled
    pub async fn fifo_xl_dual_fsm_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_xl_dualc_batch_from_fsm(val);
        fifo_ctrl2.write(self).await
    }
    /// Get if FSM-triggered batching in FIFO of accelerometer channel 2 is enabled. Required dual channel mode enabled.
    pub async fn fifo_xl_dual_fsm_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self)
            .await
            .map(|reg| reg.xl_dualc_batch_from_fsm())?;

        Ok(val)
    }
    /// It configures the compression algorithm to write non-compressed data at each rate.
    pub async fn fifo_compress_algo_set(
        &mut self,
        val: FifoCompressAlgo,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_uncompr_rate((val as u8) & 0x03);
        fifo_ctrl2.write(self).await
    }
    /// Get the actual compression algorithm.
    ///
    /// The compression algorithm handles the write of non-compressed data at each rate.
    pub async fn fifo_compress_algo_get(&mut self) -> Result<FifoCompressAlgo, Error<B::Error>> {
        let fifo_ctrl2 = FifoCtrl2::read(self).await?;

        let val = FifoCompressAlgo::try_from(fifo_ctrl2.uncompr_rate()).unwrap_or_default();

        Ok(val)
    }
    /// Enables ODR CHANGE virtual sensor to be batched in FIFO.
    pub async fn fifo_virtual_sens_odr_chg_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_odr_chg_en(val);
        fifo_ctrl2.write(self).await
    }
    /// Get the configuration (enable/disable) of ODR CHANGE virtual sensor to be batched in FIFO.
    pub async fn fifo_virtual_sens_odr_chg_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).await.map(|reg| reg.odr_chg_en())?;

        Ok(val)
    }
    /// Enables/Disables compression algorithm runtime.
    ///
    /// If val is 1: compression algorithm is active at runtime
    pub async fn fifo_compress_algo_real_time_set(
        &mut self,
        val: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_fifo_compr_rt_en(val);
        fifo_ctrl2.write(self).await?;

        self.operate_over_embed(async |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state).await?;
            emb_func_en_b.set_fifo_compr_en(val);
            emb_func_en_b.write(state).await
        })
        .await
    }
    /// Get the configuration (enable/disable) compression algorithm runtime.
    pub async fn fifo_compress_algo_real_time_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self)
            .await
            .map(|reg| reg.fifo_compr_rt_en())?;

        Ok(val)
    }
    /// Sensing chain FIFO stop values memorization at threshold level.
    pub async fn fifo_stop_on_wtm_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl2 = FifoCtrl2::read(self).await?;
        fifo_ctrl2.set_stop_on_wtm(val);
        fifo_ctrl2.write(self).await
    }
    /// Get the configuration (enable/disable) for sensing chain FIFO stop values memorization at threshold level.
    pub async fn fifo_stop_on_wtm_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl2::read(self).await.map(|reg| reg.stop_on_wtm())?;

        Ok(val)
    }
    /// Selects Batch Data Rate (write frequency in FIFO) for accelerometer data.
    pub async fn fifo_xl_batch_set(&mut self, val: FifoBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self).await?;
        fifo_ctrl3.set_bdr_xl((val as u8) & 0xF);
        fifo_ctrl3.write(self).await
    }
    /// Get the Batch Data Rate (write frequency in FIFO) for accelerometer data.
    pub async fn fifo_xl_batch_get(&mut self) -> Result<FifoBatch, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self).await?;
        let val = FifoBatch::try_from(fifo_ctrl3.bdr_xl()).unwrap_or_default();

        Ok(val)
    }
    /// Selects Batch Data Rate (write frequency in FIFO) for gyroscope data.
    pub async fn fifo_gy_batch_set(&mut self, val: FifoBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl3 = FifoCtrl3::read(self).await?;
        fifo_ctrl3.set_bdr_gy((val as u8) & 0x0F);
        fifo_ctrl3.write(self).await
    }
    /// Get Batch Data Rate (write frequency in FIFO) for gyroscope data.
    pub async fn fifo_gy_batch_get(&mut self) -> Result<FifoBatch, Error<B::Error>> {
        let fifo_ctrl3 = FifoCtrl3::read(self).await?;

        let val = FifoBatch::try_from(fifo_ctrl3.bdr_gy()).unwrap_or_default();

        Ok(val)
    }
    /// Set FIFO mode.
    pub async fn fifo_mode_set(&mut self, val: FifoMode) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_fifo_mode((val as u8) & 0x07);
        fifo_ctrl4.write(self).await
    }
    /// Get FIFO mode.
    pub async fn fifo_mode_get(&mut self) -> Result<FifoMode, Error<B::Error>> {
        let fifo_ctrl = FifoCtrl4::read(self).await?;
        let fifo_mode = FifoMode::try_from(fifo_ctrl.fifo_mode()).unwrap_or_default();

        Ok(fifo_mode)
    }
    /// Enables/Disables FIFO batching of EIS gyroscope output values.
    ///
    /// If val is 1 FIFO batching is enabled
    pub async fn fifo_gy_eis_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_g_eis_fifo_en(val);
        fifo_ctrl4.write(self).await
    }
    /// Get the configuration (enabled/disabled) for FIFO batching of EIS gyroscope output values.
    pub async fn fifo_gy_eis_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl4::read(self).await.map(|reg| reg.g_eis_fifo_en())?;

        Ok(val)
    }
    /// Set batch data rate (write frequency in FIFO) for temperature data.
    pub async fn fifo_temp_batch_set(&mut self, val: FifoTempBatch) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_odr_t_batch((val as u8) & 0x03);
        fifo_ctrl4.write(self).await
    }
    /// Get actual batch data rate (write frequency in FIFO) for temperature data.
    pub async fn fifo_temp_batch_get(&mut self) -> Result<FifoTempBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self).await?;

        let val = FifoTempBatch::try_from(fifo_ctrl4.odr_t_batch()).unwrap_or_default();

        Ok(val)
    }
    /// Selects decimation for timestamp batching in FIFO.
    ///
    /// Write rate will be the maximum rate between XL and GYRO BDR divided by decimation decoder.
    pub async fn fifo_timestamp_batch_set(
        &mut self,
        val: FifoTimestampBatch,
    ) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl4 = FifoCtrl4::read(self).await?;
        fifo_ctrl4.set_dec_ts_batch((val as u8) & 0x03);
        fifo_ctrl4.write(self).await
    }
    /// Get the actual decimation for timestamp batching in FIFO.
    ///
    /// Write rate will be the maximum rate between XL and GYRO BDR divided by decimation decoder.
    pub async fn fifo_timestamp_batch_get(
        &mut self,
    ) -> Result<FifoTimestampBatch, Error<B::Error>> {
        let fifo_ctrl4 = FifoCtrl4::read(self).await?;

        let val = FifoTimestampBatch::try_from(fifo_ctrl4.dec_ts_batch()).unwrap_or_default();

        Ok(val)
    }
    /// Set the threshold for the internal counter of batch events.
    ///
    /// When this counter reaches the threshold, the counter is
    /// reset and the interrupt flag is set to 1.
    pub async fn fifo_batch_counter_threshold_set(
        &mut self,
        val: u16,
    ) -> Result<(), Error<B::Error>> {
        let mut counter_bdr = CounterBdrReg::read(self).await?;
        counter_bdr.set_cnt_bdr_th(val);
        counter_bdr.write(self).await
    }
    /// Get the threshold for the internal counter of batch events.
    ///
    /// When this counter reaches the threshold, the counter is
    /// reset and the interrupt flag is set to 1.
    pub async fn fifo_batch_counter_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        let counter = CounterBdrReg::read(self).await?;

        Ok(counter.cnt_bdr_th())
    }
    /// Set the trigger for the internal counter of batch events.
    pub async fn fifo_batch_cnt_event_set(
        &mut self,
        val: FifoBatchCntEvent,
    ) -> Result<(), Error<B::Error>> {
        let mut counter_bdr_reg1 = CounterBdrReg::read(self).await?;
        counter_bdr_reg1.set_trig_counter_bdr(val as u8 & 0x03);
        counter_bdr_reg1.write(self).await
    }
    /// Get the actual trigger for the internal counter of batch events.
    pub async fn fifo_batch_cnt_event_get(&mut self) -> Result<FifoBatchCntEvent, Error<B::Error>> {
        let counter_bdr_reg1 = CounterBdrReg::read(self).await?;

        let val =
            FifoBatchCntEvent::try_from(counter_bdr_reg1.trig_counter_bdr()).unwrap_or_default();

        Ok(val)
    }
    /// Retrieves FIFO status.
    pub async fn fifo_status_get(&mut self) -> Result<FifoStatus, Error<B::Error>> {
        let status = FifoStatusReg::read(self).await?;

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
    pub async fn fifo_out_raw_get(&mut self) -> Result<FifoOutRaw, Error<B::Error>> {
        let fifo_data_out_tag = FifoDataOutTag::read(self).await?;
        let tag_sensor = Tag::try_from(fifo_data_out_tag.tag_sensor()).unwrap_or_default();
        let cnt = fifo_data_out_tag.tag_cnt();
        let data = FifoDataOutXYZ::read(self).await?;

        Ok(FifoOutRaw {
            tag: tag_sensor,
            cnt,
            data: data.0,
        })
    }
    /// Set the batching in FIFO buffer of step counter value.
    pub async fn fifo_stpcnt_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state).await?;
            emb_func_fifo_en_a.set_step_counter_fifo_en(val);
            emb_func_fifo_en_a.write(state).await
        })
        .await
    }
    /// Get the acutal batching in FIFO buffer of step counter value.
    pub async fn fifo_stpcnt_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            EmbFuncFifoEnA::read(state)
                .await
                .map(|reg| reg.step_counter_fifo_en())
        })
        .await
    }
    /// Enables/Disables batching in FIFO buffer of machine learning core results.
    pub async fn fifo_mlc_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state).await?;
            emb_func_fifo_en_a.set_mlc_fifo_en(val);
            emb_func_fifo_en_a.write(state).await
        })
        .await
    }
    /// Get the configuration (enables/disables) batching in FIFO buffer of machine learning core results.
    pub async fn fifo_mlc_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(EmbFuncFifoEnA::read)
            .await
            .map(|reg| reg.mlc_fifo_en())
    }
    /// Enables batching in FIFO buffer of machine learning core filters and features.
    pub async fn fifo_mlc_filt_batch_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_fifo_en_b = EmbFuncFifoEnB::read(state).await?;
            emb_func_fifo_en_b.set_mlc_filter_feature_fifo_en(val);
            emb_func_fifo_en_b.write(state).await
        })
        .await
    }
    /// Get the configuration (enable/disable) of batching in FIFO buffer
    /// of machine learning core filters and features.
    pub async fn fifo_mlc_filt_batch_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            EmbFuncFifoEnB::read(state)
                .await
                .map(|reg| reg.mlc_filter_feature_fifo_en())
        })
        .await
    }
    /// Enable FIFO data batching of target idx.
    pub async fn fifo_sh_batch_slave_set(
        &mut self,
        idx: u8,
        val: u8,
    ) -> Result<(), Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank).await?;

        let mut buf = [0u8; 1];
        self.read_from_register(SnsHubReg::Slv0Config as u8 + idx * 3, &mut buf)
            .await?;
        let mut slv_config = Slv0Config::from_bits(buf[0]);
        slv_config.set_batch_ext_sens_0_en(val);
        self.write_to_register(SnsHubReg::Slv0Config as u8 + idx * 3, &[slv_config.into()])
            .await?;

        self.mem_bank_set(MemBank::MainMemBank).await
    }
    /// Get the actual configuration (enable/disable) FIFO data batching of target idx.
    pub async fn fifo_sh_batch_slave_get(&mut self, idx: u8) -> Result<u8, Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank).await?;

        let mut buf = [0u8; 1];
        self.read_from_register(SnsHubReg::Slv0Config as u8 + idx * 3, &mut buf)
            .await?;
        let val: u8 = Slv0Config::from_bits(buf[0]).batch_ext_sens_0_en();

        self.mem_bank_set(MemBank::MainMemBank).await?;

        Ok(val)
    }
    /// Enable/Disable Batching in FIFO buffer of SFLP.
    pub async fn fifo_sflp_batch_set(&mut self, val: FifoSflpRaw) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_fifo_en_a = EmbFuncFifoEnA::read(state).await?;
            emb_func_fifo_en_a.set_sflp_game_fifo_en(val.game_rotation);
            emb_func_fifo_en_a.set_sflp_gravity_fifo_en(val.gravity);
            emb_func_fifo_en_a.set_sflp_gbias_fifo_en(val.gbias);
            emb_func_fifo_en_a.write(state).await
        })
        .await
    }
    /// Get the actual configuration (enable/disable) for Batching in FIFO buffer of SFLP.
    pub async fn fifo_sflp_batch_get(&mut self) -> Result<FifoSflpRaw, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let emb_func_fifo_en_a = EmbFuncFifoEnA::read(state).await?;

            let val = FifoSflpRaw {
                game_rotation: emb_func_fifo_en_a.sflp_game_fifo_en(),
                gravity: emb_func_fifo_en_a.sflp_gravity_fifo_en(),
                gbias: emb_func_fifo_en_a.sflp_gbias_fifo_en(),
            };

            Ok(val)
        })
        .await
    }
    /// Set Protocol anti-spike filters.
    pub async fn filt_anti_spike_set(&mut self, val: FiltAntiSpike) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self).await?;
        if_cfg.set_asf_ctrl((val as u8) & 0x01);
        if_cfg.write(self).await
    }
    /// Get the actual Protocol anti-spike filters.
    pub async fn filt_anti_spike_get(&mut self) -> Result<FiltAntiSpike, Error<B::Error>> {
        let if_cfg = IfCfg::read(self).await?;
        let val = FiltAntiSpike::try_from(if_cfg.asf_ctrl()).unwrap_or_default();

        Ok(val)
    }
    /// It masks DRDY and Interrupts RQ until filter settling ends.
    pub async fn filt_settling_mask_set(
        &mut self,
        val: FiltSettlingMask,
    ) -> Result<(), Error<B::Error>> {
        // Read CTRL4 register and set the drdy_mask field
        let mut ctrl4 = Ctrl4::read(self).await?;
        ctrl4.set_drdy_mask(val.drdy);
        ctrl4.write(self).await?;

        // Read EMB_FUNC_CFG register and set the irq_mask fields
        let mut emb_func_cfg = EmbFuncCfg::read(self).await?;
        emb_func_cfg.set_emb_func_irq_mask_xl_settl(val.irq_xl);
        emb_func_cfg.set_emb_func_irq_mask_g_settl(val.irq_g);
        emb_func_cfg.write(self).await?;

        // Read UI_INT_OIS register and set the drdy_mask_ois field
        let mut ui_int_ois = UiIntOis::read(self).await?;
        ui_int_ois.set_drdy_mask_ois(val.ois_drdy);
        ui_int_ois.write(self).await
    }
    /// Get the configuration for masks DRDY and Interrupts RQ.
    pub async fn filt_settling_mask_get(&mut self) -> Result<FiltSettlingMask, Error<B::Error>> {
        let emb_func_cfg = EmbFuncCfg::read(self).await?;
        let ui_int_ois = UiIntOis::read(self).await?;
        let ctrl4 = Ctrl4::read(self).await?;

        let val: FiltSettlingMask = FiltSettlingMask {
            irq_xl: emb_func_cfg.emb_func_irq_mask_xl_settl(),
            irq_g: emb_func_cfg.emb_func_irq_mask_g_settl(),
            drdy: ctrl4.drdy_mask(),
            ois_drdy: ui_int_ois.drdy_mask_ois(),
        };

        Ok(val)
    }
    /// Set the masks for DRDY and Interrupts RQ until filter settling ends.
    pub async fn filt_ois_settling_mask_set(
        &mut self,
        ois_drdy: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut spi2_int_ois = Spi2IntOis::read(self).await?;
        spi2_int_ois.set_drdy_mask_ois(ois_drdy);
        spi2_int_ois.write(self).await
    }
    /// Get the masks DRDY and Interrupts RQ until filter settling ends.
    pub async fn filt_ois_settling_mask_get(
        &mut self,
    ) -> Result<FiltOisSettlingMask, Error<B::Error>> {
        let spi2_int_ois = Spi2IntOis::read(self).await?;
        let val = FiltOisSettlingMask {
            ois_drdy: spi2_int_ois.drdy_mask_ois(),
        };
        Ok(val)
    }
    /// Set the Gyroscope low-pass filter (LPF1) bandwidth
    pub async fn filt_gy_lp1_bandwidth_set(
        &mut self,
        val: FiltGyLp1Bandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl6 = Ctrl6::read(self).await?;
        ctrl6.set_lpf1_g_bw((val as u8) & 0x0F);
        ctrl6.write(self).await
    }
    /// Get the gyroscope low-pass filter (LPF1) bandwidth.
    pub async fn filt_gy_lp1_bandwidth_get(
        &mut self,
    ) -> Result<FiltGyLp1Bandwidth, Error<B::Error>> {
        let ctrl6 = Ctrl6::read(self).await?;
        let bandwidth = FiltGyLp1Bandwidth::try_from(ctrl6.lpf1_g_bw()).unwrap_or_default();

        Ok(bandwidth)
    }
    /// Enables/Disable gyroscope digital LPF1 filter.
    pub async fn filt_gy_lp1_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self).await?;
        ctrl7.set_lpf1_g_en(val);
        ctrl7.write(self).await
    }
    /// Get the configuration (enables/disables) gyroscope digital LPF1 filter.
    pub async fn filt_gy_lp1_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ctrl7::read(self).await.map(|reg| reg.lpf1_g_en())
    }
    /// Set the Accelerometer LPF2 and high pass filter configuration and cutoff setting.
    pub async fn filt_xl_lp2_bandwidth_set(
        &mut self,
        val: FiltXlLp2Bandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl8 = Ctrl8::read(self).await?;
        ctrl8.set_hp_lpf2_xl_bw((val as u8) & 0x07);
        ctrl8.write(self).await
    }
    /// Get the current Accelerometer LPF2 and high pass filter configuration and cutoff setting.
    pub async fn filt_xl_lp2_bandwidth_get(
        &mut self,
    ) -> Result<FiltXlLp2Bandwidth, Error<B::Error>> {
        let ctrl8 = Ctrl8::read(self).await?;
        let bandwidth =
            FiltXlLp2Bandwidth::try_from(ctrl8.hp_lpf2_xl_bw() & 0x07).unwrap_or_default();

        Ok(bandwidth)
    }
    /// Enable/Disable accelerometer LPS2 (Low Pass Filter 2) filtering stage.
    pub async fn filt_xl_lp2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self).await?;
        ctrl9.set_lpf2_xl_en(val);
        ctrl9.write(self).await
    }
    /// Get the accelerometer LPS2 (Low Pass Filter 2) filtering stage.
    pub async fn filt_xl_lp2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).await.map(|reg| reg.lpf2_xl_en())?;

        Ok(val)
    }
    /// Accelerometer slope filter / high-pass filter selection.
    pub async fn filt_xl_hp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self).await?;
        ctrl9.set_hp_slope_xl_en(val);
        ctrl9.write(self).await
    }
    /// Get the Accelerometer slope filter / high-pass filter selection.
    pub async fn filt_xl_hp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).await.map(|reg| reg.hp_slope_xl_en())?;

        Ok(val)
    }
    /// Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the first sample.
    pub async fn filt_xl_fast_settling_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self).await?;
        ctrl9.set_xl_fastsettl_mode(val);
        ctrl9.write(self).await
    }
    /// Get accelerometer LPF2 and HPF fast-settling mode. The filter sets the first sample.
    pub async fn filt_xl_fast_settling_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl9::read(self).await.map(|reg| reg.xl_fastsettl_mode())?;

        Ok(val)
    }
    /// Set Accelerometer high-pass filter mode.
    pub async fn filt_xl_hp_mode_set(&mut self, val: FiltXlHpMode) -> Result<(), Error<B::Error>> {
        let mut ctrl9 = Ctrl9::read(self).await?;
        ctrl9.set_hp_ref_mode_xl(val as u8 & 0x01);
        ctrl9.write(self).await
    }
    /// Get Accelerometer high-pass filter mode.
    pub async fn filt_xl_hp_mode_get(&mut self) -> Result<FiltXlHpMode, Error<B::Error>> {
        let ctrl9 = Ctrl9::read(self).await?;

        let mode = FiltXlHpMode::try_from(ctrl9.hp_ref_mode_xl()).unwrap_or_default();

        Ok(mode)
    }
    /// Filter wakeup activity feed set.
    ///
    /// Set HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions.
    pub async fn filt_wkup_act_feed_set(
        &mut self,
        val: FiltWkupActFeed,
    ) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self).await?;
        let mut tap_cfg0 = TapCfg0::read(self).await?;

        tap_cfg0.set_slope_fds((val as u8) & 0x01);
        tap_cfg0.write(self).await?;

        wake_up_ths.set_usr_off_on_wu(((val as u8) & 0x02) >> 1);
        wake_up_ths.write(self).await
    }
    /// Filter wakeup activity feed get.
    ///
    /// Get the actual HPF or SLOPE filter on wake-up and Activity/Inactivity functions.
    pub async fn filt_wkup_act_feed_get(&mut self) -> Result<FiltWkupActFeed, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self).await?;
        let tap_cfg0 = TapCfg0::read(self).await?;

        let result =
            FiltWkupActFeed::try_from((wake_up_ths.usr_off_on_wu() << 1) + tap_cfg0.slope_fds())
                .unwrap_or_default();

        Ok(result)
    }
    /// Mask hw function triggers when xl is settling.
    ///
    /// If val is 1 it enables the masking
    pub async fn mask_trigger_xl_settl_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self).await?;
        tap_cfg0.set_hw_func_mask_xl_settl(val & 0x01);
        tap_cfg0.write(self).await
    }
    /// Get current configuration (enable/disable) of mask hw function
    ///
    /// triggers when xl is settling.
    pub async fn mask_trigger_xl_settl_get(&mut self) -> Result<u8, Error<B::Error>> {
        TapCfg0::read(self)
            .await
            .map(|reg| reg.hw_func_mask_xl_settl())
    }
    /// Configure the LPF2 filter on 6D (sixd) function.
    pub async fn filt_sixd_feed_set(&mut self, val: FiltSixdFeed) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self).await?;
        tap_cfg0.set_low_pass_on_6d((val as u8) & 0x01);
        tap_cfg0.write(self).await
    }
    /// Get the actual LPF2 filter on 6D (sixd) function selection.
    pub async fn filt_sixd_feed_get(&mut self) -> Result<FiltSixdFeed, Error<B::Error>> {
        let reg = TapCfg0::read(self).await?;
        let val = FiltSixdFeed::try_from(reg.low_pass_on_6d()).unwrap_or_default();
        Ok(val)
    }
    /// Set the gyroscope digital LPF_EIS filter bandwidth.
    pub async fn filt_gy_eis_lp_bandwidth_set(
        &mut self,
        val: FiltGyEisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ctrl_eis = CtrlEis::read(self).await?;
        ctrl_eis.set_lpf_g_eis_bw(val as u8 & 0x01);
        ctrl_eis.write(self).await
    }
    /// Get the current gyroscope digital LPF_EIS filter bandwidth selection.
    pub async fn filt_gy_eis_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltGyEisLpBandwidth, Error<B::Error>> {
        let ctrl_eis = CtrlEis::read(self).await?;

        let val = FiltGyEisLpBandwidth::try_from(ctrl_eis.lpf_g_eis_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Set the Gyroscope OIS digital LPF1 filter bandwidth.
    ///
    /// This function works also on OIS interface.
    pub async fn filt_gy_ois_lp_bandwidth_set(
        &mut self,
        val: FiltGyOisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl2_ois = UiCtrl2Ois::read(self).await?;

        ui_ctrl2_ois.set_lpf1_g_ois_bw((val as u8) & 0x03);
        ui_ctrl2_ois.write(self).await
    }
    /// Get the current gyroscope OIS digital LPF1 filter bandwidth.
    ///
    /// This function works also on OIS interface.
    pub async fn filt_gy_ois_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltGyOisLpBandwidth, Error<B::Error>> {
        let ui_ctrl2_ois = UiCtrl2Ois::read(self).await?;

        let val = FiltGyOisLpBandwidth::try_from(ui_ctrl2_ois.lpf1_g_ois_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Set accelerometer OIS channel bandwidth.
    ///
    /// This function works also on OIS interface.
    pub async fn filt_xl_ois_lp_bandwidth_set(
        &mut self,
        val: FiltXlOisLpBandwidth,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl3_ois = UiCtrl3Ois::read(self).await?;
        ui_ctrl3_ois.set_lpf_xl_ois_bw((val as u8) & 0x07);
        ui_ctrl3_ois.write(self).await
    }
    /// Get accelerometer OIS channel bandwidth.
    ///
    /// This function works also on OIS interface.
    pub async fn filt_xl_ois_lp_bandwidth_get(
        &mut self,
    ) -> Result<FiltXlOisLpBandwidth, Error<B::Error>> {
        let reg = UiCtrl3Ois::read(self).await?;

        let val = FiltXlOisLpBandwidth::try_from(reg.lpf_xl_ois_bw()).unwrap_or_default();

        Ok(val)
    }
    /// Enables the control of the CTRL registers to FSM.
    ///
    /// Warning: FSM can change some configurations of the device autonomously.
    pub async fn fsm_permission_set(&mut self, val: FsmPermission) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self).await?;
        func_cfg_access.set_fsm_wr_ctrl_en(val as u8 & 0x01);
        func_cfg_access.write(self).await
    }
    /// Get the FSM permission to change the CTRL registers.
    pub async fn fsm_permission_get(&mut self) -> Result<FsmPermission, Error<B::Error>> {
        let func_cfg_access = FuncCfgAccess::read(self).await?;

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
    pub async fn fsm_permission_status(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlStatus::read(self)
            .await
            .map(|reg| reg.fsm_wr_ctrl_status())?;

        Ok(val)
    }
    /// Enable Finite State Machine (FSM) feature.
    pub async fn fsm_mode_set(&mut self, val: FsmMode) -> Result<(), Error<B::Error>> {
        const PROPERTY_ENABLE: u8 = 1;
        const PROPERTY_DISABLE: u8 = 0;

        self.operate_over_embed(async |state| {
            let mut emb_func_en_b = EmbFuncEnB::read(state).await?;
            let mut fsm_enable = FsmEnable::read(state).await?;

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

            fsm_enable.write(state).await?;
            emb_func_en_b.write(state).await
        })
        .await
    }
    /// Enable Finite State Machine (FSM) feature.
    pub async fn fsm_mode_get(&mut self) -> Result<FsmMode, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let fsm_enable = FsmEnable::read(state).await?;
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
        .await
    }
    /// Set FSM long counter status register.
    ///
    /// Long counter value is an unsigned integer value (16-bit format).
    pub async fn fsm_long_cnt_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| FsmLongCounter(val).write(state).await)
            .await
    }
    /// Get FSM long counter status register.
    ///
    /// Long counter value is an unsigned integer value (16-bit format).
    pub async fn fsm_long_cnt_get(&mut self) -> Result<u16, Error<B::Error>> {
        self.operate_over_embed(FsmLongCounter::read)
            .await
            .map(|reg| reg.0)
    }
    /// Get the FSM output results.
    pub async fn fsm_out_get(&mut self) -> Result<[FsmOutsElement; 8], Error<B::Error>> {
        self.operate_over_embed(FsmOut::read).await.map(|reg| reg.0)
    }
    /// Set the Finite State Machine Output Data Rate (ODR).
    pub async fn fsm_data_rate_set(&mut self, val: FsmDataRate) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut fsm_odr = FsmOdr::read(state).await?;
            fsm_odr.set_fsm_odr((val as u8) & 0x07);
            fsm_odr.write(state).await
        })
        .await
    }
    /// Get the finite State Machine Output Data Rate (ODR) configuration.
    pub async fn fsm_data_rate_get(&mut self) -> Result<FsmDataRate, Error<B::Error>> {
        // Set the memory bank to EmbedFuncMemBank
        self.operate_over_embed(FsmOdr::read)
            .await
            .map(|reg| FsmDataRate::try_from(reg.fsm_odr()).unwrap_or_default())
    }
    /// Set SFLP GBIAS value for x/y/z axis.
    ///
    /// The register value is expressed as half-precision
    /// floating-point format: SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent
    /// bits; F: 10 fraction bits).
    pub async fn sflp_game_gbias_set(&mut self, val: &SflpGbias) -> Result<(), Error<B::Error>> {
        let mut emb_func_en_saved: (EmbFuncEnA, EmbFuncEnB);
        let mut gbias_hf: [u16; 3] = [0; 3];
        let mut data_tmp: i32;
        let mut data_bytes: [u8; 4];

        let sflp_odr = self.sflp_data_rate_get().await?;

        let k = match sflp_odr {
            SflpDataRate::_15hz => 0.04,
            SflpDataRate::_30hz => 0.02,
            SflpDataRate::_60hz => 0.01,
            SflpDataRate::_120hz => 0.005,
            SflpDataRate::_240hz => 0.0025,
            SflpDataRate::_480hz => 0.00125,
        };

        // compute gbias as half precision float in order to be put in embedded advanced feature register
        gbias_hf[0] = from_single_precision_to_half(val.gbias_x * (core::f32::consts::PI / 180.0) / k);
        gbias_hf[1] = from_single_precision_to_half(val.gbias_y * (core::f32::consts::PI / 180.0) / k);
        gbias_hf[2] = from_single_precision_to_half(val.gbias_z * (core::f32::consts::PI / 180.0) / k);

        // Save sensor configuration and set high-performance mode (if the sensor is in the power-down
        // mode, turn it on)
        let conf_saved = (Ctrl1::read(self).await?, Ctrl2::read(self).await?);
        self.xl_mode_set(XlMode::HighPerformanceMd).await?;
        self.gy_mode_set(GyMode::HighPerformanceMd).await?;
        if conf_saved.0.odr_xl() == Odr::Off as u8 {
            self.xl_data_rate_set(Odr::_120hz).await?;
        }

        // make sure to turn the sensor-hub master off
        let master_config = self.sh_master_get().await?;
        self.sh_master_set(0).await?;

        // disable algos
        emb_func_en_saved = self
            .operate_over_embed(async |state| {
                let tmp_emb_func_en_saved = (
                    EmbFuncEnA::read(state).await?,
                    EmbFuncEnB::read(state).await?,
                );
                EmbFuncEnA::from_bits(0).write(state).await?;
                EmbFuncEnB::from_bits(0).write(state).await?;
                loop {
                    let emb_func_sts = EmbFuncExecStatus::read(state).await?;
                    if emb_func_sts.emb_func_endop() == 1 {
                        break;
                    }
                }

                Ok(tmp_emb_func_en_saved)
            })
            .await?;

        // enable gbias setting
        let mut ctrl10 = Ctrl10::read(self).await?;
        ctrl10.set_emb_func_debug(1);
        ctrl10.write(self).await?;

        // enable algos
        self.operate_over_embed(async |state| {
            emb_func_en_saved.0.set_sflp_game_en(1); // force SFLP GAME en
            emb_func_en_saved.0.write(state).await?;
            emb_func_en_saved.1.write(state).await
        })
        .await?;

        let xl_fs = self.xl_full_scale_get().await?;

        loop {
            let drdy = self.flag_data_ready_get().await?;
            if drdy.drdy_xl == 1 {
                break;
            }
        }

        let xl_data: [i16; 3] = self.acceleration_raw_get().await?;

        // force sflp initialization
        self.mem_bank_set(MemBank::SensorHubMemBank).await?;
        for i in 0..3 {
            data_tmp = xl_data[i as usize] as i32;
            data_tmp <<= xl_fs as i32; // shift based on current fs
            data_bytes = data_tmp.to_le_bytes();
            self.write_to_register(SnsHubReg::SensorHub1 as u8 + 3 * i, &[data_bytes[0]])
                .await?;
            self.write_to_register(SnsHubReg::SensorHub2 as u8 + 3 * i, &[data_bytes[1]])
                .await?;
            self.write_to_register(SnsHubReg::SensorHub3 as u8 + 3 * i, &[data_bytes[2]])
                .await?;
        }
        for i in 0..3 {
            data_tmp = 0;
            data_bytes = data_tmp.to_le_bytes();
            self.write_to_register(SnsHubReg::SensorHub10 as u8 + 3 * i, &[data_bytes[0]])
                .await?;
            self.write_to_register(SnsHubReg::SensorHub11 as u8 + 3 * i, &[data_bytes[1]])
                .await?;
            self.write_to_register(SnsHubReg::SensorHub12 as u8 + 3 * i, &[data_bytes[2]])
                .await?;
        }
        self.mem_bank_set(MemBank::MainMemBank).await?;

        // wait end op (and at least 30 us)
        self.tim.delay_ms(1).await;
        self.operate_over_embed(async |state| {
            loop {
                let emb_func_sts = EmbFuncExecStatus::read(state).await?;
                if emb_func_sts.emb_func_endop() == 1 {
                    break;
                }
            }
            Ok(())
        })
        .await?;

        // write gbias in embedded advanced features registers
        SflpGameGbiasXYZ(gbias_hf).write(self).await?;

        // reload previous sensor configuration
        conf_saved.0.write(self).await?;
        conf_saved.1.write(self).await?;

        // disable gbias setting
        ctrl10.set_emb_func_debug(0);
        ctrl10.write(self).await?;

        // reload previous master configuration
        self.sh_master_set(master_config).await
    }
    /// Set the External sensor sensitivity value register for the Finite State Machine.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x1624 (when using an external magnetometer this value
    /// corresponds to 0.0015 gauss/LSB).
    pub async fn fsm_ext_sens_sensitivity_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmExtSensitivity(val).write(self).await
    }
    /// Get the External sensor sensitivity value register for the Finite State Machine.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x1624 (when using an external magnetometer this value
    /// corresponds to 0.0015 gauss/LSB).
    pub async fn fsm_ext_sens_sensitivity_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmExtSensitivity::read(self).await.map(|reg| reg.0)
    }
    /// Set External sensor offsets (X,Y,Z).
    ///
    /// The values are expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub async fn fsm_ext_sens_offset_set(
        &mut self,
        val: XlFsmExtSensOffset,
    ) -> Result<(), Error<B::Error>> {
        FsmExtOffXYZ([val.x, val.y, val.z]).write(self).await
    }
    /// Get External sensor offsets (X,Y,Z).
    ///
    /// The values are expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub async fn fsm_ext_sens_offset_get(&mut self) -> Result<XlFsmExtSensOffset, Error<B::Error>> {
        FsmExtOffXYZ::read(self)
            .await
            .map(|reg| XlFsmExtSensOffset {
                x: reg.0[0],
                y: reg.0[1],
                z: reg.0[2],
            })
    }
    /// Set External sensor transformation matrix.
    ///
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub async fn fsm_ext_sens_matrix_set(
        &mut self,
        val: XlFsmExtSensMatrix,
    ) -> Result<(), Error<B::Error>> {
        let buff: [u8; 12] = val.to_le_bytes();

        FsmExtMatrix(buff).write(self).await
    }
    /// Get the External sensor transformation matrix.
    ///
    /// The value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    pub async fn fsm_ext_sens_matrix_get(&mut self) -> Result<XlFsmExtSensMatrix, Error<B::Error>> {
        let buff: [u8; 12] = FsmExtMatrix::read(self).await?.0;
        Ok(XlFsmExtSensMatrix::from_le_bytes(buff))
    }
    /// Set External sensor z-axis coordinates rotation.
    pub async fn fsm_ext_sens_z_orient_set(
        &mut self,
        val: FsmExtSensZOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_a = ExtCfgA::read(self).await?;
        ext_cfg_a.set_ext_z_axis((val as u8) & 0x07);
        ext_cfg_a.write(self).await
    }
    /// Get External sensor z-axis coordinates rotation.
    pub async fn fsm_ext_sens_z_orient_get(
        &mut self,
    ) -> Result<FsmExtSensZOrient, Error<B::Error>> {
        let ext_cfg_a = ExtCfgA::read(self).await?;

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
    pub async fn fsm_ext_sens_y_orient_set(
        &mut self,
        val: FsmExtSensYOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_a = ExtCfgA::read(self).await?;
        ext_cfg_a.set_ext_y_axis((val as u8) & 0x7);
        ext_cfg_a.write(self).await
    }
    /// Get External sensor Y-axis coordinates rotation.
    pub async fn fsm_ext_sens_y_orient_get(
        &mut self,
    ) -> Result<FsmExtSensYOrient, Error<B::Error>> {
        let ext_cfg_a = ExtCfgA::read(self).await?;
        let val = FsmExtSensYOrient::try_from(ext_cfg_a.ext_y_axis()).unwrap_or_default();

        Ok(val)
    }
    /// Set External sensor X-axis coordinates rotation.
    pub async fn fsm_ext_sens_x_orient_set(
        &mut self,
        val: FsmExtSensXOrient,
    ) -> Result<(), Error<B::Error>> {
        let mut ext_cfg_b = ExtCfgB::read(self).await?;
        ext_cfg_b.set_ext_x_axis((val as u8) & 0x7);
        ext_cfg_b.write(self).await
    }
    /// Get External sensor X-axis coordinates rotation.
    pub async fn fsm_ext_sens_x_orient_get(
        &mut self,
    ) -> Result<FsmExtSensXOrient, Error<B::Error>> {
        let ext_cfg_b = ExtCfgB::read(self).await?;
        let val = FsmExtSensXOrient::try_from(ext_cfg_b.ext_x_axis()).unwrap_or_default();

        Ok(val)
    }
    /// Set FSM long counter timeout.
    ///
    /// The long counter timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reaches this value, the FSM generates an interrupt.
    pub async fn fsm_long_cnt_timeout_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmLcTimeout(val).write(self).await
    }
    /// Get FSM long counter timeout.
    ///
    /// The long counter timeout value is an unsigned integer value (16-bit format).
    /// When the long counter value reached this value, the FSM generates an interrupt.
    pub async fn fsm_long_cnt_timeout_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmLcTimeout::read(self).await.map(|reg| reg.0)
    }
    /// Set the FSM number of programs.
    ///
    /// Must be less than or equal to 8. Default 0.
    pub async fn fsm_number_of_programs_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut fsm_programs = FsmPrograms::read(self).await?;
        fsm_programs.set_fsm_n_prog(val);
        fsm_programs.write(self).await
    }
    /// Get the actual FSM number of programs.
    pub async fn fsm_number_of_programs_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FsmPrograms::read(self).await?.fsm_n_prog();
        Ok(val)
    }
    /// Set the FSM start address.
    ///
    /// First available address is 0x35C.
    pub async fn fsm_start_address_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        FsmStartAdd(val).write(self).await
    }
    /// Get the actual FSM start address.
    ///
    /// First available address is 0x35C.
    pub async fn fsm_start_address_get(&mut self) -> Result<u16, Error<B::Error>> {
        FsmStartAdd::read(self).await.map(|reg| reg.0)
    }
    /// Set the time windows configuration for Free Fall detection.
    ///
    /// 1 LSB = 1/ODR_XL time
    pub async fn ff_time_windows_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        // Read WAKE_UP_DUR and configure wake_up_dur
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_ff_dur((val & 0x20) >> 5);
        wake_up_dur.write(self).await?;

        // Read FREE_FALL and configure free_fall
        let mut free_fall = FreeFall::read(self).await?;
        free_fall.set_ff_dur(val & 0x1F);
        free_fall.write(self).await
    }
    /// Get the time windows configuration for Free Fall detection.
    ///
    /// 1 LSB = 1/ODR_XL time
    pub async fn ff_time_windows_get(&mut self) -> Result<u8, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self).await?;
        let free_fall = FreeFall::read(self).await?;

        let val: u8 = (wake_up_dur.ff_dur() << 5) + free_fall.ff_dur();

        Ok(val)
    }
    /// Set the Free fall threshold.
    pub async fn ff_thresholds_set(&mut self, val: FfThreshold) -> Result<(), Error<B::Error>> {
        let mut free_fall = FreeFall::read(self).await?;
        free_fall.set_ff_ths((val as u8) & 0x7);

        free_fall.write(self).await
    }
    /// Get the current Free fall threshold setting.
    pub async fn ff_thresholds_get(&mut self) -> Result<FfThreshold, Error<B::Error>> {
        let free_fall = FreeFall::read(self).await?;

        let val = FfThreshold::try_from(free_fall.ff_ths()).unwrap_or_default();

        Ok(val)
    }
    /// Set Machine Learning Core mode (MLC).
    ///registermod
    /// When the Machine Learning Core is enabled the Finite State Machine (FSM)
    /// programs are executed before executing the MLC algorithms.
    pub async fn mlc_set(&mut self, val: MlcMode) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_en_a = EmbFuncEnA::read(state).await?;
            let mut emb_en_b = EmbFuncEnB::read(state).await?;

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

            emb_en_a.write(state).await?;
            emb_en_b.write(state).await
        })
        .await
    }
    /// Get the configuration ofMachine Learning Core (MLC).
    ///
    /// When the Machine Learning Core is enabled the Finite State Machine (FSM)
    /// programs are executed before executing the MLC algorithms.
    pub async fn mlc_get(&mut self) -> Result<MlcMode, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let emb_en_a = EmbFuncEnA::read(state).await?;
            let emb_en_b = EmbFuncEnB::read(state).await?;

            let val = match (emb_en_a.mlc_before_fsm_en(), emb_en_b.mlc_en()) {
                (0, 0) => MlcMode::Off,
                (0, 1) => MlcMode::On,
                (1, _) => MlcMode::OnBeforeFsm,
                _ => MlcMode::Off,
            };

            Ok(val)
        })
        .await
    }
    /// Set Machine Learning Core Output Data Rate (ODR).
    pub async fn mlc_data_rate_set(&mut self, val: MlcDataRate) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut mlc_odr = MlcOdr::read(state).await?;
            mlc_odr.set_mlc_odr((val as u8) & 0x07);
            mlc_odr.write(state).await
        })
        .await
    }
    /// Get the Machine Learning Core Output Data Rate (ODR).
    pub async fn mlc_data_rate_get(&mut self) -> Result<MlcDataRate, Error<B::Error>> {
        self.operate_over_embed(MlcOdr::read)
            .await
            .map(|reg| MlcDataRate::try_from(reg.mlc_odr()).unwrap_or_default())
    }
    /// Get the output value of all MLC decision trees.
    pub async fn mlc_out_get(&mut self) -> Result<MlcOut, Error<B::Error>> {
        self.operate_over_embed(MlcSrc::read)
            .await
            .map(|reg| MlcOut::from_le_bytes(reg.0))
    }
    /// Set the External sensor sensitivity value register for the Machine Learning Core.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x3C00 (when using an external magnetometer this value
    /// corresponds to 1 gauss/LSB).
    pub async fn mlc_ext_sens_sensitivity_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        MlcExtSensitivity(val).write(self).await
    }
    /// Get the External sensor sensitivity value register for the Machine Learning Core.
    ///
    /// This register corresponds to the conversion value of the external sensor.
    /// The register value is expressed as half-precision floating-point format:
    /// SEEEEEFFFFFFFFFF (S: 1 sign bit; E: 5 exponent bits; F: 10 fraction bits).
    /// Default value is 0x3C00 (when using an external magnetometer this value
    /// corresponds to 1 gauss/LSB).
    pub async fn mlc_ext_sens_sensitivity_get(&mut self) -> Result<u16, Error<B::Error>> {
        MlcExtSensitivity::read(self).await.map(|reg| reg.0)
    }
    /// Get the configuration for the full control of OIS configurations from the UI (User Interface).
    pub async fn ois_ctrl_mode_set(&mut self, val: OisCtrlMode) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self).await?;
        func_cfg_access.set_ois_ctrl_from_ui((val as u8) & 0x1);
        func_cfg_access.write(self).await
    }
    /// Get the configuration for the full control of OIS configurations from the UI (User Interface).
    pub async fn ois_ctrl_mode_get(&mut self) -> Result<OisCtrlMode, Error<B::Error>> {
        let func_cfg_access = FuncCfgAccess::read(self).await?;

        let val = OisCtrlMode::try_from(func_cfg_access.ois_ctrl_from_ui()).unwrap_or_default();

        Ok(val)
    }
    /// Resets the control registers of OIS from the UI (User Interface).
    ///
    /// The bits set are not auto-cleared.
    pub async fn ois_reset_set(&mut self, val: i8) -> Result<(), Error<B::Error>> {
        let mut func_cfg_access = FuncCfgAccess::read(self).await?;

        func_cfg_access.set_spi2_reset(val as u8);

        func_cfg_access.write(self).await
    }
    /// Get Resets the control registers of OIS from the UI (User Interface)
    pub async fn ois_reset_get(&mut self) -> Result<i8, Error<B::Error>> {
        let val: i8 = FuncCfgAccess::read(self)
            .await
            .map(|reg| reg.spi2_reset() as i8)?;

        Ok(val)
    }
    /// Enable/disable pull up on OIS interface.
    pub async fn ois_interface_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self).await?;
        pin_ctrl.set_ois_pu_dis(val);
        pin_ctrl.write(self).await
    }
    /// Get the configuration (enable/disable) pull up on OIS interface.
    pub async fn ois_interface_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PinCtrl::read(self).await.map(|reg| reg.ois_pu_dis())?;

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
    pub async fn ois_handshake_from_ui_set(
        &mut self,
        val: OisHandshake,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_handshake_ctrl = UiHandshakeCtrl::read(self).await?;
        ui_handshake_ctrl.set_ui_shared_ack(val.ack);
        ui_handshake_ctrl.set_ui_shared_req(val.req);
        ui_handshake_ctrl.write(self).await
    }
    /// Get Handshake for UI (User Interface) SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake.
    /// If the secondary interface is not accessing the shared registers, this bit is set to 1 by the device and the R/W operation
    /// on the UI_SPI2_SHARED registers is allowed on the primary interface. REQ: This bit is used by the primary interface master
    /// to request access to the UI_SPI2_SHARED registers. When the R/W operation is finished, the master must reset this bit.
    pub async fn ois_handshake_from_ui_get(&mut self) -> Result<OisHandshake, Error<B::Error>> {
        let ui_handshake_ctrl: UiHandshakeCtrl = UiHandshakeCtrl::read(self).await?;
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
    pub async fn ois_handshake_from_ois_set(
        &mut self,
        val: OisHandshake,
    ) -> Result<(), Error<B::Error>> {
        let mut spi2_handshake_ctrl: Spi2HandshakeCtrl = Spi2HandshakeCtrl::read(self).await?;
        spi2_handshake_ctrl.set_spi2_shared_ack(val.ack);
        spi2_handshake_ctrl.set_spi2_shared_req(val.req);
        spi2_handshake_ctrl.write(self).await
    }
    /// Get Handshake for OIS interface SPI2 shared registers.
    ///
    /// ACK: This bit acknowledges the handshake. If the secondary interface is not accessing
    /// the shared registers, this bit is set to 1 by the device and the R/W operation on the
    /// UI_SPI2_SHARED registers is allowed on the primary interface.
    /// REQ: This bit is used by the primary interface master to request access to the
    /// UI_SPI2_SHARED registers. When the R/W operation is finished, the master must reset this bit.
    pub async fn ois_handshake_from_ois_get(&mut self) -> Result<OisHandshake, Error<B::Error>> {
        let spi2_handshake_ctrl = Spi2HandshakeCtrl::read(self).await?;
        let val = OisHandshake {
            ack: spi2_handshake_ctrl.spi2_shared_ack(),
            req: spi2_handshake_ctrl.spi2_shared_req(),
        };

        Ok(val)
    }
    /// Set User interface (UI) / SPI2 (OIS) shared registers
    pub async fn ois_shared_set(&mut self, val: [u8; 6]) -> Result<(), Error<B::Error>> {
        UiSpi2Shared(val).write(self).await
    }
    /// Get User interface (UI) / SPI2 (OIS) shared registers
    pub async fn ois_shared_get(&mut self) -> Result<[u8; 6], Error<B::Error>> {
        UiSpi2Shared::read(self).await.map(|reg| reg.0)
    }
    /// Enables SPI2 (OIS Interface) for reading OIS data when User Interface (UI) is in full control mode
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub async fn ois_on_spi2_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self).await?;
        ui_ctrl1_ois.set_spi2_read_en(val);
        ui_ctrl1_ois.write(self).await
    }
    /// Get configuration (enable/disable) of SPI2 (OIS Interface) for reading OIS data when
    /// User Interface (UI) is in full control mode
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub async fn ois_on_spi2_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = UiCtrl1Ois::read(self).await.map(|reg| reg.spi2_read_en())?;

        Ok(val)
    }
    /// Enables gyroscope/accelerometer OIS chain.
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub async fn ois_chain_set(&mut self, val: OisChain) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self).await?;
        ui_ctrl1_ois.set_ois_g_en(val.gy);
        ui_ctrl1_ois.set_ois_xl_en(val.xl);
        ui_ctrl1_ois.write(self).await
    }
    /// Get the configuration (enable/disable) gyroscope/accelerometer OIS chain.
    pub async fn ois_chain_get(&mut self) -> Result<OisChain, Error<B::Error>> {
        let ui_ctrl1_ois = UiCtrl1Ois::read(self).await?;
        let gy = ui_ctrl1_ois.ois_g_en();
        let xl = ui_ctrl1_ois.ois_xl_en();

        let val = OisChain { gy, xl };

        Ok(val)
    }
    /// Set gyroscope OIS full-scale
    pub async fn ois_gy_full_scale_set(
        &mut self,
        val: OisGyFullScale,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl2_ois = UiCtrl2Ois::read(self).await?;
        ui_ctrl2_ois.set_fs_g_ois(val as u8 & 0x03);
        ui_ctrl2_ois.write(self).await
    }
    /// Get the gyroscope OIS full-scale selection
    pub async fn ois_gy_full_scale_get(&mut self) -> Result<OisGyFullScale, Error<B::Error>> {
        let fs_g_ois = UiCtrl2Ois::read(self).await.map(|reg| reg.fs_g_ois())?;

        let val = OisGyFullScale::try_from(fs_g_ois).unwrap_or_default();

        Ok(val)
    }
    /// Set accelerometer OIS channel full-scale.
    pub async fn ois_xl_full_scale_set(
        &mut self,
        val: OisXlFullScale,
    ) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl3_ois = UiCtrl3Ois::read(self).await?;
        ui_ctrl3_ois.set_fs_xl_ois((val as u8) & 0x3);
        ui_ctrl3_ois.write(self).await
    }
    /// Get accelerometer OIS channel full-scale.
    pub async fn ois_xl_full_scale_get(&mut self) -> Result<OisXlFullScale, Error<B::Error>> {
        let ui_ctrl3_ois = UiCtrl3Ois::read(self).await?;

        let val = OisXlFullScale::try_from(ui_ctrl3_ois.fs_xl_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Set Threshold for 4D/6D function.
    pub async fn threshold_6d_set(&mut self, val: SixDThreshold) -> Result<(), Error<B::Error>> {
        let mut tap_ths_6d = TapThs6d::read(self).await?;
        tap_ths_6d.set_sixd_ths((val as u8) & 0x03);
        tap_ths_6d.write(self).await
    }
    /// Get Threshold for 4D/6D function.
    pub async fn threshold_6d_get(&mut self) -> Result<SixDThreshold, Error<B::Error>> {
        let tap_ths_6d = TapThs6d::read(self).await?;

        let value = SixDThreshold::try_from(tap_ths_6d.sixd_ths()).unwrap_or_default();

        Ok(value)
    }
    /// Enables/Disables 4D orientation detection.
    ///
    /// Z-axis position detection is disabled.
    pub async fn mode_4d_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = TapThs6d::read(self).await?;
        reg.set_d4d_en(val);
        reg.write(self).await
    }
    /// Get the configuration for 4D orientation detection enable.
    ///
    /// Z-axis position detection is disabled.
    pub async fn mode_4d_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = TapThs6d::read(self).await.map(|reg| reg.d4d_en())?;

        Ok(val)
    }
    /// Configures the equivalent input impedance of the AH_QVAR buffers.
    pub async fn ah_qvar_zin_set(&mut self, val: AhQvarZin) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self).await?;
        ctrl7.set_ah_qvar_c_zin((val as u8) & 0x03);
        ctrl7.write(self).await
    }
    /// Get the actual input impedance of the AH_QVAR buffers.
    pub async fn ah_qvar_zin_get(&mut self) -> Result<AhQvarZin, Error<B::Error>> {
        let ctrl7 = Ctrl7::read(self).await?;

        let result = AhQvarZin::try_from(ctrl7.ah_qvar_c_zin()).unwrap_or_default();

        Ok(result)
    }
    /// Enables AH_QVAR chain.
    ///
    /// When this bit is set to '1', the AH_QVAR buffers are connected to the SDx/AH1/Qvar1 and SCx/AH2/Qvar2 pins.
    /// Before setting this bit to 1, the accelerometer and gyroscope sensor have to be configured in power-down mode.
    pub async fn ah_qvar_mode_set(&mut self, val: AhQvarMode) -> Result<(), Error<B::Error>> {
        let mut ctrl7 = Ctrl7::read(self).await?;
        ctrl7.set_ah_qvar_en(val.ah_qvar_en);
        ctrl7.write(self).await
    }
    /// Get configuration (enable/disable) for AH_QVAR chain.
    ///
    /// When this bit is set to '1', the AH_QVAR buffers are connected to the SDx/AH1/Qvar1 and SCx/AH2/Qvar2 pins.
    /// Before setting this bit to 1, the accelerometer and gyroscope sensor have to be configured in power-down mode.
    pub async fn ah_qvar_mode_get(&mut self) -> Result<AhQvarMode, Error<B::Error>> {
        let ctrl7 = Ctrl7::read(self).await?;
        let val = AhQvarMode {
            ah_qvar_en: ctrl7.ah_qvar_en(),
        };

        Ok(val)
    }
    /// Set the action the device will perform after "Reset whole chip" I3C pattern.
    pub async fn i3c_reset_mode_set(&mut self, val: I3cResetMode) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self).await?;
        pin_ctrl.set_ibhr_por_en((val as u8) & 0x01);
        pin_ctrl.write(self).await
    }
    /// Get the i3c reset mode.
    ///
    /// After set the action the device will perform after "Reset whole chip" I3C pattern.
    pub async fn i3c_reset_mode_get(&mut self) -> Result<I3cResetMode, Error<B::Error>> {
        let pin_ctrl = PinCtrl::read(self).await?;
        let mode = I3cResetMode::try_from(pin_ctrl.ibhr_por_en()).unwrap_or_default();
        Ok(mode)
    }
    /// Enable/Disable INT pin when I3C is used.
    pub async fn i3c_int_en_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl5 = Ctrl5::read(self).await?;
        ctrl5.set_int_en_i3c(val & 0x01);
        ctrl5.write(self).await
    }
    /// Get configuration (enable/disable) INT pin when I3C is used.
    pub async fn i3c_int_en_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Ctrl5::read(self).await.map(|reg| reg.int_en_i3c())?;

        Ok(val)
    }
    /// Set the us activity time for IBI (In-Band Interrupt) with I3C.
    pub async fn i3c_ibi_time_set(&mut self, val: I3cIbiTime) -> Result<(), Error<B::Error>> {
        let mut ctrl5 = Ctrl5::read(self).await?;
        ctrl5.set_bus_act_sel((val as u8) & 0x03);
        ctrl5.write(self).await
    }
    /// Get the us activity time for IBI (In-Band Interrupt) with I3C.
    pub async fn i3c_ibi_time_get(&mut self) -> Result<I3cIbiTime, Error<B::Error>> {
        let ctrl5 = Ctrl5::read(self).await?;

        let val = I3cIbiTime::try_from(ctrl5.bus_act_sel()).unwrap_or_default();

        Ok(val)
    }
    /// Enable/Disable Sensor Hub master I2C pull-up.
    pub async fn sh_master_interface_pull_up_set(
        &mut self,
        val: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self).await?;
        if_cfg.set_shub_pu_en(val);
        if_cfg.write(self).await
    }
    /// Get Sensor Hub master I2C pull-up configuration (enable/disable).
    pub async fn sh_master_interface_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = IfCfg::read(self).await.map(|reg| reg.shub_pu_en())?;

        Ok(val)
    }
    /// Sensor hub output registers.
    ///
    /// The length of the array input determines the length of the read.
    /// Valid range goes from 0..18
    pub async fn sh_read_data_raw_get(&mut self, rbuf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| SensorHub1::read_more(state, rbuf).await)
            .await
    }
    /// Set the number of external sensors to be read by the sensor hub.
    pub async fn sh_slave_connected_set(
        &mut self,
        val: ShSlaveConnected,
    ) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_aux_sens_on((val as u8) & 0x3);
            master_config.write(state).await
        })
        .await
    }
    /// Get the number of external sensors to be read by the sensor hub.
    pub async fn sh_slave_connected_get(&mut self) -> Result<ShSlaveConnected, Error<B::Error>> {
        self.operate_over_sensorhub(MasterConfig::read)
            .await
            .map(|reg| ShSlaveConnected::try_from(reg.aux_sens_on()).unwrap_or_default())
    }
    /// Enable/Disable Sensor hub I2C master configuration.
    pub async fn sh_master_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_master_on(val);
            master_config.write(state).await
        })
        .await
    }
    /// Get Sensor hub I2C master configuration (enable/disable).
    pub async fn sh_master_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_sensorhub(MasterConfig::read)
            .await
            .map(|reg| reg.master_on())
    }
    /// Enable/Disable I2C interface pass-through.
    pub async fn sh_pass_through_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        // Set the memory bank to SensorHubMemBank
        self.operate_over_sensorhub(async |state| {
            // Modify the MasterConfig register
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_pass_through_mode(val);
            master_config.write(state).await
        })
        .await
    }
    /// Get I2C interface pass-through configuration (enable/disable).
    pub async fn sh_pass_through_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_sensorhub(MasterConfig::read)
            .await
            .map(|reg| reg.pass_through_mode())
    }
    /// Set Sensor hub trigger signal configuration.
    pub async fn sh_syncro_mode_set(&mut self, val: ShSyncroMode) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_start_config((val as u8) & 0x01);
            master_config.write(state).await
        })
        .await
    }
    /// Get Sensor hub trigger signal configuration.
    pub async fn sh_syncro_mode_get(&mut self) -> Result<ShSyncroMode, Error<B::Error>> {
        self.operate_over_sensorhub(MasterConfig::read)
            .await
            .map(|reg| ShSyncroMode::try_from(reg.start_config()).unwrap_or_default())
    }
    /// Set Slave 0 write operation mode.
    ///
    /// Permit switch write mode between: only at the first sensor hub cycle or at each sensor hub cycle.
    pub async fn sh_write_mode_set(&mut self, val: ShWriteMode) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_write_once(val as u8 & 0x01);
            master_config.write(state).await
        })
        .await
    }
    /// Get Slave 0 write operation mode.
    pub async fn sh_write_mode_get(&mut self) -> Result<ShWriteMode, Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let write_once = MasterConfig::read(state).await?.write_once();
            let mode = ShWriteMode::try_from(write_once).unwrap_or_default();
            Ok(mode)
        })
        .await
    }
    /// Reset Master logic and output registers.
    ///
    /// Must be set to '1' and then set it to '0'.
    pub async fn sh_reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut master_config = MasterConfig::read(state).await?;
            master_config.set_rst_master_regs(val);
            master_config.write(state).await
        })
        .await
    }
    /// Get Reset Master logic and output registers.
    ///
    /// Must be set to '1' and then set it to '0'.
    pub async fn sh_reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_sensorhub(MasterConfig::read)
            .await
            .map(|reg| reg.rst_master_regs())
    }
    /// Configure slave 0 to perform a write.
    pub async fn sh_cfg_write(&mut self, val: ShCfgWrite) -> Result<(), Error<B::Error>> {
        // Set the memory bank to SensorHubMemBank
        self.operate_over_sensorhub(async |state| {
            let mut reg = Slv0Add::read(state).await?;
            reg.set_slave0_add(val.slv0_add);
            reg.set_rw_0(0);
            // Write to the Slv0Add register
            reg.write(state).await?;

            // Write to the DatawriteSlv0 register
            let data_write_slv0 = DatawriteSlv0::from_bits(val.slv0_data);
            data_write_slv0.write(state).await
        })
        .await
    }
    /// Set the rate at which the master communicates.
    pub async fn sh_data_rate_set(&mut self, val: ShDataRate) -> Result<(), Error<B::Error>> {
        self.operate_over_sensorhub(async |state| {
            let mut slv0_config = Slv0Config::read(state).await?;

            slv0_config.set_shub_odr((val as u8) & 0x07);
            slv0_config.write(state).await
        })
        .await
    }
    /// Get the rate at which the controller communicates.
    pub async fn sh_data_rate_get(&mut self) -> Result<ShDataRate, Error<B::Error>> {
        self.operate_over_sensorhub(Slv0Config::read)
            .await
            .map(|reg| ShDataRate::try_from(reg.shub_odr()).unwrap_or_default())
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
    pub async fn sh_slv_cfg_read(
        &mut self,
        idx: u8,
        val: &ShCfgRead,
    ) -> Result<(), Error<B::Error>> {
        self.mem_bank_set(MemBank::SensorHubMemBank).await?;
        let mut arr: [u8; 1] = [0];
        let mut slv_add = Slv0Add::from_bits(arr[0]);
        slv_add.set_slave0_add(val.slv_add);
        slv_add.set_rw_0(1);

        self.write_to_register(SnsHubReg::Slv0Add as u8 + (idx * 3), &[slv_add.into_bits()])
            .await?;

        self.write_to_register(SnsHubReg::Slv0Subadd as u8 + (idx * 3), &[val.slv_subadd])
            .await?;

        self.read_from_register(SnsHubReg::Slv0Config as u8 + (idx * 3), &mut arr)
            .await?;
        let mut slv_config = Slv0Config::from_bits(arr[0]);
        slv_config.set_slave0_numop(val.slv_len);

        self.write_to_register(
            SnsHubReg::Slv0Config as u8 + (idx * 3),
            &[slv_config.into_bits()],
        )
        .await?;

        self.mem_bank_set(MemBank::MainMemBank).await
    }

    /// Get Sensor hub status register.
    pub async fn sh_status_get(&mut self) -> Result<StatusMaster, Error<B::Error>> {
        let status = StatusMasterMainPage::read(self).await?;
        Ok(StatusMaster::from_bits(status.into_bits()))
    }
    /// Enables/Disables pull-up on SDO pin of UI (User Interface).
    pub async fn ui_sdo_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pin_ctrl = PinCtrl::read(self).await?;
        pin_ctrl.set_sdo_pu_en(val);
        pin_ctrl.write(self).await
    }
    /// Get the pull-up on SDO pin of UI (User Interface) configuration.
    pub async fn ui_sdo_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PinCtrl::read(self).await.map(|reg| reg.sdo_pu_en())?;

        Ok(val)
    }
    /// Enables/Disables I2C and I3C on UI (User Interface).
    pub async fn ui_i2c_i3c_mode_set(&mut self, val: UiI2cI3cMode) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self).await?;
        if_cfg.set_i2c_i3c_disable((val as u8) & 0x1);
        if_cfg.write(self).await
    }
    /// Get I2C and I3C on UI (User Interface) mode configuration.
    pub async fn ui_i2c_i3c_mode_get(&mut self) -> Result<UiI2cI3cMode, Error<B::Error>> {
        let reg = IfCfg::read(self).await?;

        let val = UiI2cI3cMode::try_from(reg.i2c_i3c_disable()).unwrap_or_default();

        Ok(val)
    }
    /// SPI Serial Interface Mode selection.
    pub async fn spi_mode_set(&mut self, val: SpiMode) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self).await?;
        if_cfg.set_sim((val as u8) & 0x01);
        if_cfg.write(self).await
    }
    /// Get the SPI Serial Interface Mode.
    pub async fn spi_mode_get(&mut self) -> Result<SpiMode, Error<B::Error>> {
        let if_cfg = IfCfg::read(self).await?;

        let val = SpiMode::try_from(if_cfg.sim()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables pull-up on SDA pin.
    pub async fn ui_sda_pull_up_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut if_cfg = IfCfg::read(self).await?;
        if_cfg.set_sda_pu_en(val);
        if_cfg.write(self).await
    }
    /// Get pull-up configuration on SDA pin.
    pub async fn ui_sda_pull_up_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = IfCfg::read(self).await.map(|reg| reg.sda_pu_en())?;

        Ok(val)
    }
    /// Select SPI2 (OIS Interface) Serial Interface Mode.
    ///
    /// This function works also on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub async fn spi2_mode_set(&mut self, val: Spi2Mode) -> Result<(), Error<B::Error>> {
        let mut ui_ctrl1_ois = UiCtrl1Ois::read(self).await?;
        ui_ctrl1_ois.set_sim_ois((val as u8) & 0x01);
        ui_ctrl1_ois.write(self).await
    }
    /// Get SPI2 (OIS Interface) Serial Interface Mode.
    /// This function also works on OIS (UI_CTRL1_OIS = SPI2_CTRL1_OIS).
    pub async fn spi2_mode_get(&mut self) -> Result<Spi2Mode, Error<B::Error>> {
        let ui_ctrl1_ois = UiCtrl1Ois::read(self).await?;

        let val = Spi2Mode::try_from(ui_ctrl1_ois.sim_ois()).unwrap_or_default();

        Ok(val)
    }
    /// Enables/Disables significant motion detection function.
    pub async fn sigmot_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state).await?;
            emb_func_en_a.set_sign_motion_en(val);
            emb_func_en_a.write(state).await
        })
        .await
    }
    /// Get significant motion detection function configuration (enable/disable).
    pub async fn sigmot_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(async |state| {
            EmbFuncEnA::read(state)
                .await
                .map(|reg| reg.sign_motion_en())
        })
        .await
    }
    /// Set Step counter mode.
    pub async fn stpcnt_mode_set(&mut self, val: StpcntMode) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state).await?;
            let emb_func_en_b = EmbFuncEnB::read(state).await?;

            if val.false_step_rej == 1
                && emb_func_en_a.mlc_before_fsm_en() & emb_func_en_b.mlc_en() == 0
            {
                emb_func_en_a.set_mlc_before_fsm_en(1);
            }

            emb_func_en_a.set_pedo_en(val.step_counter_enable);
            emb_func_en_a.write(state).await
        })
        .await?;

        let mut pedo_cmd_reg = PedoCmdReg::read(self).await?;
        pedo_cmd_reg.set_fp_rejection_en(val.false_step_rej);
        pedo_cmd_reg.write(self).await
    }
    /// Get Step counter mode
    pub async fn stpcnt_mode_get(&mut self) -> Result<StpcntMode, Error<B::Error>> {
        let emb_func_en_a = self.operate_over_embed(EmbFuncEnA::read).await?;
        let pedo_cmd_reg = PedoCmdReg::read(self).await?;

        let val = StpcntMode {
            false_step_rej: pedo_cmd_reg.fp_rejection_en(),
            step_counter_enable: emb_func_en_a.pedo_en(),
        };

        Ok(val)
    }
    /// Get Step counter output: number of detected steps.
    pub async fn stpcnt_steps_get(&mut self) -> Result<u16, Error<B::Error>> {
        self.operate_over_embed(StepCounter::read)
            .await
            .map(|reg| reg.0)
    }
    /// Reset step counter.
    ///
    /// If val is 1: step counter is reset
    pub async fn stpcnt_rst_step_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_src: EmbFuncSrc = EmbFuncSrc::read(state).await?;
            emb_func_src.set_pedo_rst_step(val);
            emb_func_src.write(state).await
        })
        .await
    }
    /// Get reset step counter.
    pub async fn stpcnt_rst_step_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(EmbFuncSrc::read)
            .await
            .map(|reg| reg.pedo_rst_step())
    }
    /// Set Pedometer debounce number.
    pub async fn stpcnt_debounce_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut pedo_deb_steps_conf = PedoDebStepsConf::read(self).await?;
        pedo_deb_steps_conf.set_deb_step(val);
        pedo_deb_steps_conf.write(self).await
    }
    /// Get Pedometer debounce number.
    pub async fn stpcnt_debounce_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = PedoDebStepsConf::read(self).await?.deb_step();

        Ok(val)
    }
    /// Set time period register for step detection on delta time.
    pub async fn stpcnt_period_set(&mut self, val: u16) -> Result<(), Error<B::Error>> {
        let reg = PedoScDeltaT(val);
        reg.write(self).await
    }
    /// Get time period register for step detection on delta time.
    pub async fn stpcnt_period_get(&mut self) -> Result<u16, Error<B::Error>> {
        PedoScDeltaT::read(self).await.map(|reg| reg.0)
    }
    /// Enables/Disables SFLP Game Rotation Vector (6x).
    pub async fn sflp_game_rotation_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state).await?;
            emb_func_en_a.set_sflp_game_en(val);
            emb_func_en_a.write(state).await
        })
        .await
    }
    /// Get the configuration (enable/disable) for SFLP Game Rotation Vector (6x).
    pub async fn sflp_game_rotation_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(EmbFuncEnA::read)
            .await
            .map(|reg| reg.sflp_game_en())
    }
    /// Set SFLP Data Rate (ODR).
    pub async fn sflp_data_rate_set(&mut self, val: SflpDataRate) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut sflp_odr = SflpOdr::read(state).await?;
            sflp_odr.set_sflp_game_odr((val as u8) & 0x07);
            sflp_odr.write(state).await
        })
        .await
    }
    /// Get SFLP Data Rate (ODR).
    pub async fn sflp_data_rate_get(&mut self) -> Result<SflpDataRate, Error<B::Error>> {
        self.operate_over_embed(SflpOdr::read)
            .await
            .map(|reg| SflpDataRate::try_from(reg.sflp_game_odr()).unwrap_or_default())
    }
    /// Enable axis for Tap - Double Tap detection.
    pub async fn tap_detection_set(&mut self, val: TapDetection) -> Result<(), Error<B::Error>> {
        let mut tap_cfg0 = TapCfg0::read(self).await?;
        tap_cfg0.set_tap_x_en(val.tap_x_en);
        tap_cfg0.set_tap_y_en(val.tap_y_en);
        tap_cfg0.set_tap_z_en(val.tap_z_en);
        tap_cfg0.write(self).await
    }
    /// Get configuration for Tap on each axis - Double Tap detection.
    pub async fn tap_detection_get(&mut self) -> Result<TapDetection, Error<B::Error>> {
        let tap_cfg0 = TapCfg0::read(self).await?;

        let tap_detection = TapDetection {
            tap_x_en: tap_cfg0.tap_x_en(),
            tap_y_en: tap_cfg0.tap_y_en(),
            tap_z_en: tap_cfg0.tap_z_en(),
        };

        Ok(tap_detection)
    }
    /// Set Double Tap recognition thresholds - axis Tap
    pub async fn tap_thresholds_set(&mut self, val: TapThresholds) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self).await?;
        let mut tap_cfg2 = TapCfg2::read(self).await?;
        let mut tap_ths_6d = TapThs6d::read(self).await?;

        tap_cfg1.set_tap_ths_x(val.x);
        tap_cfg2.set_tap_ths_y(val.y);
        tap_ths_6d.set_tap_ths_z(val.z);

        tap_ths_6d.write(self).await?;
        tap_cfg2.write(self).await?;
        tap_cfg1.write(self).await
    }
    /// Get Double Tap recognition thresholds - axis Tap
    pub async fn tap_thresholds_get(&mut self) -> Result<TapThresholds, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self).await?;
        let tap_cfg2 = TapCfg2::read(self).await?;
        let tap_ths_6d = TapThs6d::read(self).await?;

        let thresholds = TapThresholds {
            // Dummy values or replace with appropriate method calls
            x: tap_cfg1.tap_ths_x(),
            y: tap_cfg2.tap_ths_y(),
            z: tap_ths_6d.tap_ths_z(),
        };

        Ok(thresholds)
    }
    /// Set axis priority for TAP detection.
    pub async fn tap_axis_priority_set(
        &mut self,
        val: TapAxisPriority,
    ) -> Result<(), Error<B::Error>> {
        let mut tap_cfg1 = TapCfg1::read(self).await?;
        tap_cfg1.set_tap_priority((val as u8) & 0x7);
        tap_cfg1.write(self).await
    }
    /// Get axis priority for TAP detection.
    pub async fn tap_axis_priority_get(&mut self) -> Result<TapAxisPriority, Error<B::Error>> {
        let tap_cfg1 = TapCfg1::read(self).await?;

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
    pub async fn tap_time_windows_set(
        &mut self,
        val: TapTimeWindows,
    ) -> Result<(), Error<B::Error>> {
        let mut tap_dur = TapDur::read(self).await?;
        tap_dur.set_shock(val.shock);
        tap_dur.set_quiet(val.quiet);
        tap_dur.set_dur(val.tap_gap);
        tap_dur.write(self).await
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
    pub async fn tap_time_windows_get(&mut self) -> Result<TapTimeWindows, Error<B::Error>> {
        let tap_dur = TapDur::read(self).await?;

        let val = TapTimeWindows {
            shock: tap_dur.shock(),
            quiet: tap_dur.quiet(),
            tap_gap: tap_dur.dur(),
        };

        Ok(val)
    }
    /// Enable/Disable single/double-tap event.
    pub async fn tap_mode_set(&mut self, val: TapMode) -> Result<(), Error<B::Error>> {
        let mut wake_up_ths = WakeUpThs::read(self).await?;
        wake_up_ths.set_single_double_tap((val as u8) & 0x01);
        wake_up_ths.write(self).await
    }
    /// Get configuration (enable/disable) for Single/double-tap event
    pub async fn tap_mode_get(&mut self) -> Result<TapMode, Error<B::Error>> {
        let wake_up_ths = WakeUpThs::read(self).await?;

        let val = TapMode::try_from(wake_up_ths.single_double_tap()).unwrap_or_default();

        Ok(val)
    }
    /// Set Tilt mode.
    pub async fn tilt_mode_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        self.operate_over_embed(async |state| {
            let mut emb_func_en_a = EmbFuncEnA::read(state).await?;
            emb_func_en_a.set_tilt_en(val);
            emb_func_en_a.write(state).await
        })
        .await
    }
    /// Get Tilt mode.
    pub async fn tilt_mode_get(&mut self) -> Result<u8, Error<B::Error>> {
        self.operate_over_embed(EmbFuncEnA::read)
            .await
            .map(|reg| reg.tilt_en())
    }
    /// Get Timestamp raw data
    pub async fn timestamp_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        Timestamp::read(self).await.map(|reg| reg.0)
    }
    /// Enables timestamp counter.
    pub async fn timestamp_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self).await?;
        functions_enable.set_timestamp_en(val);
        functions_enable.write(self).await
    }
    /// Get the actual timestamp counter configuration.
    ///
    /// If return 1 timestamp counter is active
    pub async fn timestamp_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FunctionsEnable::read(self)
            .await
            .map(|reg| reg.timestamp_en())?;

        Ok(val)
    }

    /// Configure activity/inactivity (sleep)
    ///
    /// `ActMode` could handle different setting for accelerometer and gyroscope
    pub async fn act_mode_set(&mut self, val: ActMode) -> Result<(), Error<B::Error>> {
        let mut functions_enable = FunctionsEnable::read(self).await?;
        functions_enable.set_inact_en(val as u8 & 0x3);
        functions_enable.write(self).await
    }
    /// Get activity/inactivity (sleep)
    pub async fn act_mode_get(&mut self) -> Result<ActMode, Error<B::Error>> {
        let functions_enable = FunctionsEnable::read(self).await?;

        let val = ActMode::try_from(functions_enable.inact_en()).unwrap_or_default();

        Ok(val)
    }
    /// Set duration in the transition from Stationary to Motion (from Inactivity to Activity).
    pub async fn act_from_sleep_to_act_dur_set(
        &mut self,
        val: ActFromSleepToActDur,
    ) -> Result<(), Error<B::Error>> {
        let mut inactivity_dur = InactivityDur::read(self).await?;
        inactivity_dur.set_inact_dur((val as u8) & 0x3);
        inactivity_dur.write(self).await
    }
    /// Get duration in the transition from Stationary to Motion (from Inactivity to Activity).
    pub async fn act_from_sleep_to_act_dur_get(
        &mut self,
    ) -> Result<ActFromSleepToActDur, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self).await?;

        let val = ActFromSleepToActDur::try_from(inactivity_dur.inact_dur()).unwrap_or_default();

        Ok(val)
    }
    /// Set the accelerometer data rate during Inactivity.
    pub async fn act_sleep_xl_odr_set(
        &mut self,
        val: ActSleepXlOdr,
    ) -> Result<(), Error<B::Error>> {
        let mut inactivity_dur = InactivityDur::read(self).await?;
        inactivity_dur.set_xl_inact_odr((val as u8) & 0x03);
        inactivity_dur.write(self).await
    }
    /// Get the accelerometer data rate during Inactivity.
    pub async fn act_sleep_xl_odr_get(&mut self) -> Result<ActSleepXlOdr, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self).await?;

        let val = ActSleepXlOdr::try_from(inactivity_dur.xl_inact_odr()).unwrap_or_default();

        Ok(val)
    }
    /// Set Wakeup and activity/inactivity threshold.
    pub async fn act_thresholds_set(&mut self, val: ActThresholds) -> Result<(), Error<B::Error>> {
        // Read current values from registers
        let mut inactivity_dur = InactivityDur::read(self).await?;
        let mut inactivity_ths = InactivityThs::read(self).await?;
        let mut wake_up_ths = WakeUpThs::read(self).await?;
        let mut wake_up_dur = WakeUpDur::read(self).await?;

        // Set new values
        inactivity_dur.set_wu_inact_ths_w(val.inactivity_cfg.wu_inact_ths_w());
        inactivity_dur.set_xl_inact_odr(val.inactivity_cfg.xl_inact_odr());
        inactivity_dur.set_inact_dur(val.inactivity_cfg.inact_dur());

        inactivity_ths.set_inact_ths(val.inactivity_ths);
        wake_up_ths.set_wk_ths(val.threshold);
        wake_up_dur.set_wake_dur(val.duration);

        // Write new values back to registers
        inactivity_dur.write(self).await?;
        inactivity_ths.write(self).await?;
        wake_up_ths.write(self).await?;
        wake_up_dur.write(self).await
    }
    /// Get Wakeup and activity/inactivity threshold.
    pub async fn act_thresholds_get(&mut self) -> Result<ActThresholds, Error<B::Error>> {
        let inactivity_dur = InactivityDur::read(self).await?;
        let inactivity_ths = InactivityThs::read(self).await?;
        let wake_up_ths = WakeUpThs::read(self).await?;
        let wake_up_dur = WakeUpDur::read(self).await?;

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
    pub async fn act_wkup_time_windows_set(
        &mut self,
        val: ActWkupTimeWindows,
    ) -> Result<(), Error<B::Error>> {
        let mut wake_up_dur = WakeUpDur::read(self).await?;
        wake_up_dur.set_wake_dur(val.shock);
        wake_up_dur.set_sleep_dur(val.quiet);
        wake_up_dur.write(self).await
    }
    /// Get Time windows configuration for Wake Up - Activity - Inactivity (SLEEP, WAKE).
    ///
    /// Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR)
    /// 1 LSB = 512/ODR_XL time. Wake up duration event. 1 LSB = 1/ODR_XL time.
    pub async fn act_wkup_time_windows_get(
        &mut self,
    ) -> Result<ActWkupTimeWindows, Error<B::Error>> {
        let wake_up_dur = WakeUpDur::read(self).await?;

        let val = ActWkupTimeWindows {
            shock: wake_up_dur.wake_dur(),
            quiet: wake_up_dur.sleep_dur(),
        };

        Ok(val)
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
 * Converts a 32-bit single-precision value to a 16-bit half-precision
 * representation and returns the raw bits.
 *
 * # Description
 *
 * Uses the `half` crate to convert a native `f32` into an IEEE 754
 * half-precision (`f16`) value, and then returns its bit pattern as `u16`.
 *
 * # Parameters
 *
 * * `float`: 32-bit single-precision floating-point value to be converted.
 *
 * # Return
 *
 * * `u16`: Bit pattern of the corresponding IEEE 754 half-precision value.
 */
pub fn from_single_precision_to_half(float: f32) -> u16 {
    f16::from_f32(float).to_bits()
}

/*
 * Converts a 16-bit half-precision floating-point value to a 32-bit
 * single-precision representation.
 *
 * # Description
 *
 * Interprets the input `u16` as an IEEE 754 half-precision (`f16`) bit pattern,
 * and converts it to a native `f32` using the `half` crate
 *
 * # Parameters
 *
 * * `half`: 16-bit value containing the half-precision float bit pattern.
 *
 * # Returns
 *
 * * `f32`: 16-bit value in single-precision representation.
 */
pub fn from_half_to_single_precision(half: u16) -> f32 {
    let half = f16::from_bits(half);
    half.to_f32()
}

#[cfg(feature = "passthrough")]
/// Lsm6dsv16xPassthrough
///
/// Encapsulate the sensor as a bus that could be used inside another sensor
/// to provide passthrough capability
pub struct Lsm6dsv16xPassthrough<'a, B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    sensor: &'a mut Lsm6dsv16x<B, T, MainBank>,
    slave_address: SevenBitAddress,
}

#[cfg(feature = "passthrough")]
impl<'a, B, T> Lsm6dsv16xPassthrough<'a, B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    pub fn new_from_sensor(
        sensor: &'a mut Lsm6dsv16x<B, T, MainBank>,
        slave_address: SevenBitAddress,
    ) -> Self {
        Lsm6dsv16xPassthrough {
            sensor,
            slave_address,
        }
    }
}

#[cfg(feature = "passthrough")]
// LSM6DSV16X acts like a bus when used for the sensor hub.
#[bisync]
impl<B, T> BusOperation for Lsm6dsv16xPassthrough<'_, B, T>
where
    B: BusOperation,
    T: DelayNs,
{
    type Error = Error<B::Error>;

    async fn read_bytes(&mut self, _rbuf: &mut [u8]) -> Result<(), Self::Error> {
        Err(Error::UnexpectedValue)
    }

    async fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        let master = &mut self.sensor;
        for i in 1_u8..(wbuf.len() as u8) {
            // Configure Sensor Hub to read data
            let sh_cfg_write = ShCfgWrite {
                slv0_add: self.slave_address,
                slv0_subadd: wbuf[0] + i - 1,
                slv0_data: wbuf[i as usize],
            };
            master.sh_cfg_write(sh_cfg_write).await?;

            // Disable accelerometer
            master.xl_data_rate_set(Odr::Off).await?;
            // Enable I2C Master
            master.sh_master_set(1).await?;
            // Enable accelerometer to trigger Sensor Hub operation.
            master.xl_data_rate_set(Odr::_120hz).await?;
            // Wait Sensor Hub operation flag set.
            master.acceleration_raw_get().await?; // dummy read

            let mut drdy = 0;
            while drdy == 0 {
                master.tim.delay_ms(20).await;
                drdy = master.flag_data_ready_get().await?.drdy_xl;
            }

            let mut end_op = 0;
            while end_op == 0 {
                master.tim.delay_ms(20).await;
                end_op = master.sh_status_get().await?.sens_hub_endop();
            }

            // Disable I2C master and XL (triger).
            master.sh_master_set(0).await?;
            master.xl_data_rate_set(Odr::Off).await?;
        }

        Ok(())
    }

    async fn write_byte_read_bytes(
        &mut self,
        wbuf: &[u8; 1],
        rbuf: &mut [u8],
    ) -> Result<(), Self::Error> {
        let master = &mut self.sensor;
        // Disable accelerometer
        master.xl_data_rate_set(Odr::Off).await?;
        // Configure Sensor Hub to read
        let sh_cfg_read = ShCfgRead {
            slv_add: self.slave_address,
            slv_subadd: wbuf[0],
            slv_len: rbuf.len() as u8,
        };
        master.sh_slv_cfg_read(0, &sh_cfg_read).await?; // dummy read
        master.sh_slave_connected_set(ShSlaveConnected::_01).await?;
        // Enable I2C Master
        master.sh_master_set(1).await?;
        // Enable accelerometer to trigger Sensor Hub operation.
        master.xl_data_rate_set(Odr::_120hz).await?;
        // Wait Sensor Hub operation flag set
        master.acceleration_raw_get().await?; // dummy read

        let mut drdy = 0;
        while drdy == 0 {
            master.tim.delay_ms(20).await;
            drdy = master.flag_data_ready_get().await?.drdy_xl;
        }

        let mut end_op = 0;
        while end_op == 0 {
            //master.tim.delay_ms(20).await;
            end_op = master.sh_status_get().await?.sens_hub_endop();
        }

        // Disable I2C master and XL(trigger)
        master.sh_master_set(0).await?;
        master.xl_data_rate_set(Odr::Off).await?;

        // Read SensorHub registers
        master.sh_read_data_raw_get(rbuf).await
    }
}

#[derive(Clone, Copy, Default)]
#[bisync]
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
#[bisync]
pub enum I2CAddress {
    I2cAddL = 0x6A,
    I2cAddH = 0x6B,
}

#[bisync]
pub const ID: u8 = 0x70;
