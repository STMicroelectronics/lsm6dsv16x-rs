#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;
use st_mems_bus;
use st_mems_bus::i2c::I2cBus;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use lis2mdl_rs as lis2mdl;
use lps22df_rs as lps22df;
use lsm6dsv16x::prelude::*;
use lsm6dsv16x::Lsm6dsv16xMaster;
use lsm6dsv16x_rs as lsm6dsv16x;
use panic_halt as _;
use stm32f4xx_hal::{
    gpio,
    gpio::{Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    interrupt, pac,
    prelude::*,
    serial::{config::Config, Serial},
};

type IntPin = gpio::PB4<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static FIFO_WATERMARK: u8 = 64;

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();

    let mut int_pin = gpiob.pb4.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI); // Enable the external interrupt in the NVIC

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    delay.delay_ms(5);

    let lsm6dsv16x_addr = lsm6dsv16x::I2CAddress::I2cAddH;
    let lis2mdl_addr = lis2mdl::I2CAddress::I2cAdd as u8;
    let lps22df_addr = lps22df::I2CAddress::I2cAddH as u8;

    let ref_tim = &RefCell::new(tim1);

    let master = Lsm6dsv16xMaster::from_bus(
        I2cBus::new(i2c, lsm6dsv16x_addr as u8),
        st_mems_bus::Shared::new(&ref_tim),
    );

    let lsm6dsv16x_for_lis2mdl = master.as_passthrough(lis2mdl_addr);
    let mut lis2mdl = lis2mdl::Lis2mdl::new_bus(lsm6dsv16x_for_lis2mdl);

    let lsm6dsv16x_for_lps22df = master.as_passthrough(lps22df_addr);
    let mut lps22df =
        lps22df::Lps22df::new_bus(lsm6dsv16x_for_lps22df, st_mems_bus::Shared::new(&ref_tim));

    // Check device ID
    match master.borrow_mut().device_id_get() {
        Ok(id) => {
            if id != lsm6dsv16x::ID {
                writeln!(tx, "Device (LSM6DSV16X) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    master
        .borrow_mut()
        .reset_set(Reset::RestoreCtrlRegs)
        .unwrap();
    let mut rst = Reset::GlobalRst;
    while rst != Reset::Ready {
        rst = master.borrow_mut().reset_get().unwrap();
    }

    // Enable Block Data Update
    master.borrow_mut().block_data_update_set(1).unwrap();
    // Set full scale
    master
        .borrow_mut()
        .xl_full_scale_set(XlFullScale::_2g)
        .unwrap();

    /*
     * Configure LIS2MDL target
     */

    // Check if LI2MDL connected to Sensor Hub
    match lis2mdl.device_id_get() {
        Ok(id) => {
            if id != lis2mdl::ID {
                writeln!(tx, "Device (LIS2MDL) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    lis2mdl.reset_set(1).unwrap();

    loop {
        let rst = lis2mdl.reset_get().unwrap();
        if rst == 0 {
            break;
        }
    }

    lis2mdl.block_data_update_set(1).unwrap();
    lis2mdl.offset_temp_comp_set(1).unwrap();
    lis2mdl
        .operating_mode_set(lis2mdl::prelude::Md::ContinuousMode)
        .unwrap();
    lis2mdl.data_rate_set(lis2mdl::prelude::Odr::_20hz).unwrap();
    /*
     * Configure LPS22DF target
     */

    // Check if LPS22DF connected to Sensor Hub.
    match lps22df.id_get() {
        Ok(id) => {
            if id != lps22df::ID {
                writeln!(tx, "Device (LPS22DF) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    lps22df.init_set(lps22df::prelude::Init::Reset).unwrap();
    loop {
        let status = lps22df.status_get().unwrap();
        if status.sw_reset == 0 {
            break;
        }
    }

    // Set bdu and if if_inc recommeded for driver usage
    lps22df.init_set(lps22df::prelude::Init::DrvRdy).unwrap();

    // Select bus interface
    let mut bus_mode = lps22df::prelude::BusMode::default();
    bus_mode.filter = lps22df::prelude::Filter::FilterAuto;
    bus_mode.interface = lps22df::prelude::Interface::SelByHw;
    lps22df.bus_mode_set(&bus_mode).unwrap();

    // Set Output Data Rate
    let mut md = lps22df::prelude::Md::default();
    md.odr = lps22df::prelude::Odr::_25hz;
    md.avg = lps22df::prelude::Avg::_16;
    md.lpf = lps22df::prelude::LowPassFilter::OdrDiv4;
    lps22df.mode_set(&md).unwrap();

    /*
     *  Slave settings ended: take direct ownership of the master
     */
    // drop to ensure no other borrow_mut from slaves, otherwise borrow_mut()
    // call should be called for every function call of the master
    drop(lps22df);
    drop(lis2mdl);

    // Slave instances not more used, take the ownership of the master
    let mut lsm6dsv16x = master.borrow_mut();

    /*
     * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to FIFO_WATERMARK samples
     */
    lsm6dsv16x.fifo_watermark_set(FIFO_WATERMARK).unwrap();

    /* Set FIFO batch XL/Gyro ODR to 60hz */
    lsm6dsv16x.fifo_xl_batch_set(FifoBatch::_60hz).unwrap();

    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dsv16x.fifo_mode_set(FifoMode::StreamMode).unwrap();

    let mut pin_int = PinIntRoute::default();
    pin_int.fifo_th = 1;
    lsm6dsv16x.pin_int1_route_set(&pin_int).unwrap();

    // Set Output Data Rate
    lsm6dsv16x.xl_data_rate_set(Odr::_60hz).unwrap();
    lsm6dsv16x
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec32)
        .unwrap();
    lsm6dsv16x.timestamp_set(1).unwrap();

    // Set full scale
    lsm6dsv16x.xl_full_scale_set(XlFullScale::_2g).unwrap();

    // Configure filtering chain
    let mut filt_settling_mask = FiltSettlingMask::default();
    filt_settling_mask.drdy = 1;
    filt_settling_mask.irq_xl = 1;
    filt_settling_mask.irq_g = 1;
    lsm6dsv16x
        .filt_settling_mask_set(filt_settling_mask)
        .unwrap();
    lsm6dsv16x.filt_xl_lp2_set(1).unwrap();
    lsm6dsv16x
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .unwrap();

    lsm6dsv16x.xl_data_rate_set(Odr::Off).unwrap();

    /*
     * Prepare sensor hub to read data from external slave0 (lis2mdl) and
     * slave1 (lps22df) continuously in order to store data in FIFO.
     */

    let mut sh_cfg_read = ShCfgRead::default();
    sh_cfg_read.slv_add = lis2mdl_addr;
    sh_cfg_read.slv_subadd = lis2mdl::prelude::Reg::OutxLReg as u8;
    sh_cfg_read.slv_len = 6;
    lsm6dsv16x.sh_slv_cfg_read(0, &sh_cfg_read).unwrap();
    lsm6dsv16x.fifo_sh_batch_slave_set(0, 1).unwrap();

    sh_cfg_read.slv_add = lps22df_addr;
    sh_cfg_read.slv_subadd = lps22df::prelude::Reg::PressOutXl as u8;
    sh_cfg_read.slv_len = 6;
    lsm6dsv16x.sh_slv_cfg_read(1, &sh_cfg_read).unwrap();
    lsm6dsv16x.fifo_sh_batch_slave_set(1, 1).unwrap();

    // Configure Sensor Hub data rate
    lsm6dsv16x.sh_data_rate_set(ShDataRate::_60hz).unwrap();

    // Configure Sensor Hub to read one slave.
    lsm6dsv16x
        .sh_slave_connected_set(ShSlaveConnected::_01)
        .unwrap();

    // Set SHUB write_once bit
    lsm6dsv16x
        .sh_write_mode_set(ShWriteMode::OnlyFirstCycle)
        .unwrap();

    // Enable I2C Master
    lsm6dsv16x.sh_master_set(1).unwrap();

    /* Set Output Data Rate.
     * Selected data rate have to be equal or greater with respect
     * with MLC data rate.
     */
    lsm6dsv16x.xl_data_rate_set(Odr::_120hz).unwrap();

    let mut x: i16;
    let mut y: i16;
    let mut z: i16;
    let mut ts: u32;

    loop {
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }
        // Read watermark flag
        let fifo_status = lsm6dsv16x.fifo_status_get().unwrap();
        let num = fifo_status.fifo_level;

        writeln!(tx, "-- FIFO num {}", num).unwrap();

        for _ in 0..num {
            // Read FIFO sensor value
            let f_data = lsm6dsv16x.fifo_out_raw_get().unwrap();
            x = ((f_data.data[1] as i16) << 8) | f_data.data[0] as i16;
            y = ((f_data.data[3] as i16) << 8) | f_data.data[2] as i16;
            z = ((f_data.data[5] as i16) << 8) | f_data.data[4] as i16;
            ts = ((y as u32) << 16) | (x as u32); // 32bit buffer read

            match f_data.tag {
                Tag::XlNcTag => {
                    let acc_x = lsm6dsv16x::from_fs2_to_mg(x);
                    let acc_y = lsm6dsv16x::from_fs2_to_mg(y);
                    let acc_z = lsm6dsv16x::from_fs2_to_mg(z);
                    writeln!(
                        tx,
                        "Acceleration [mg]:{:.2}\t{:.2}\t{:.2}",
                        acc_x, acc_y, acc_z
                    )
                    .unwrap();
                }
                Tag::TimestampTag => {
                    let ts_usec = lsm6dsv16x::from_lsb_to_nsec(ts) / 1000.0;
                    writeln!(tx, "Timestamp {:.1} [us] (lsb: {})", ts_usec, ts).unwrap();
                }
                Tag::SensorhubSlave0Tag => {
                    let mg_x = lis2mdl::from_lsb_to_mgauss(x);
                    let mg_y = lis2mdl::from_lsb_to_mgauss(y);
                    let mg_z = lis2mdl::from_lsb_to_mgauss(z);

                    writeln!(tx, "LIS2MDL [mGa]:\t{:.2}\t{:.2}\t{:.2}", mg_x, mg_y, mg_z).unwrap();
                }
                Tag::SensorhubSlave1Tag => {
                    // pressure conversion
                    let baro = f_data.data[2] as i32;
                    let baro = (baro << 8) + f_data.data[1] as i32;
                    let baro = (baro << 8) + f_data.data[0] as i32;
                    let baro = baro << 8;
                    let baro = lps22df::from_lsb_to_hpa(baro as i32);

                    // temperature conversion
                    let temp = ((f_data.data[4] as u16) << 8) + f_data.data[3] as u16;
                    let temp = lps22df::from_lsb_to_celsius(temp as i16);

                    writeln!(tx, "LPS22DF [hPa]:{:.2} [degC]{:.2}", baro, temp).unwrap();
                }
                _ => {
                    writeln!(tx, "Invalid TAG: {:x}", f_data.tag as u8).unwrap();
                }
            }
        }
    }
}

/*
 // To provide custom passthrough implementation edit this template

struct Lsm6dsv16xWrapper<B, T> {
    pub instance: Lsm6dsv16x<B, T>
}

impl<B, T> bus::BusOperation for Lsm6dsv16xWrapper<B, T> where B: bus::BusOperation, T: DelayNs {
    type Error = lsm6dsv16x::Error<B::Error>;

    fn read_bytes(&mut self, _rbuf: &mut [u8]) -> Result<(), Self::Error> {
        Err(lsm6dsv16x::Error::UnexpectedValue)
    }

    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        let mut sh_cfg_write = lsm6dsv16x::ShCfgWrite::default();

        for i in 1_u8..(wbuf.len() as u8) {
            // Configure Sensor Hub to read data
            sh_cfg_write.slv0_add = self.instance.slave_address.ok_or(lsm6dsv16x::Error::UnexpectedValue)?;
            sh_cfg_write.slv0_subadd = wbuf[0] + i - 1;
            sh_cfg_write.slv0_data = wbuf[i as usize];
            self.instance.sh_cfg_write(sh_cfg_write)?;

            // Disable accelerometer
            self.instance.xl_data_rate_set(lsm6dsv16x::Odr::Off)?;
            // Enable I2C Master
            self.instance.sh_master_set(1)?;
            // Enable accelerometer to trigger Sensor Hub operation.
            self.instance.xl_data_rate_set(lsm6dsv16x::Odr::_120hz)?;
            // Wait Sensor Hub operation flag set.
            let _dummy = self.instance.acceleration_raw_get();

            let mut drdy = 0;
            while drdy == 0 {
                self.instance.tim.delay_ms(20);
                drdy = self.instance.flag_data_ready_get()?.drdy_xl;
            }

            let mut end_op = 0;
            while end_op == 0 {
                self.instance.tim.delay_ms(20);
                end_op = self.instance.sh_status_get()?.sens_hub_endop();
            }

            // Disable I2C master and XL (triger).
            self.instance.sh_master_set(0)?;
            self.instance.xl_data_rate_set(lsm6dsv16x::Odr::Off)?;
        }

        Ok(())
    }

    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8])-> Result<(), Self::Error> {

        // Disable accelerometer
        self.instance.xl_data_rate_set(lsm6dsv16x::Odr::Off)?;
        // Configure Sensor Hub to read
        let mut sh_cfg_read = lsm6dsv16x::ShCfgRead::default();
        sh_cfg_read.slv_add = self.instance.slave_address.ok_or(lsm6dsv16x::Error::UnexpectedValue)?;
        sh_cfg_read.slv_subadd = wbuf[0];
        sh_cfg_read.slv_len = rbuf.len() as u8;
        let _dummy = self.instance.sh_slv_cfg_read(0, &sh_cfg_read)?;
        self.instance.sh_slave_connected_set(lsm6dsv16x::ShSlaveConnected::_01)?;
        // Enable I2C Master
        self.instance.sh_master_set(1)?;
        // Enable accelerometer to trigger Sensor Hub operation.
        self.instance.xl_data_rate_set(lsm6dsv16x::Odr::_120hz)?;
        // Wait Sensor Hub operation flag set
        let _dummy = self.instance.acceleration_raw_get()?;

        let mut drdy = 0;
        while drdy == 0 {
            self.instance.tim.delay_ms(20);
            drdy = self.instance.flag_data_ready_get()?.drdy_xl;
        }

        let mut end_op = 0;
        while end_op == 0 {
            //self.instance.tim.delay_ms(20);
            end_op = self.instance.sh_status_get()?.sens_hub_endop();
        }

        // Disable I2C master and XL(trigger)
        self.instance.sh_master_set(0)?;
        self.instance.xl_data_rate_set(lsm6dsv16x::Odr::Off)?;

        // Read SensorHub registers
        self.instance.sh_read_data_raw_get(rbuf)?;

        Ok(())
    }

}
*/

#[interrupt]
fn EXTI4() {
    cortex_m::interrupt::free(|cs| {
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    })
}
