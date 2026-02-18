use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[cfg(feature = "async")]
use lis2mdl_rs::asynchronous as lis2mdl;
#[cfg(feature = "async")]
use lps22df_rs::asynchronous as lps22df;

#[cfg(not(feature = "async"))]
use lis2mdl_rs::blocking as lis2mdl;
#[cfg(not(feature = "async"))]
use lps22df_rs::blocking as lps22df;

static FIFO_WATERMARK: u8 = 64;

#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, delay: D, mut int_pin: I) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
    I: InterruptPin
{
    use lsm6dsv16x::prelude::*;

    let mut sensor = Lsm6dsv16x::from_bus(bus, delay.clone());

    let lis2mdl_addr = lis2mdl::I2CAddress::I2cAdd as u8;
    let lps22df_addr = lps22df::I2CAddress::I2cAddH as u8;

    // Check device ID
    let whoami = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", whoami);
    if whoami != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor
        .reset_set(Reset::RestoreCtrlRegs)
        .await.unwrap();
    let mut rst = Reset::GlobalRst;
    while rst != Reset::Ready {
        rst = sensor.reset_get().await.unwrap();
    }


    // Enable Block Data Update
    sensor.block_data_update_set(1).await.unwrap();
    // Set full scale
    sensor
        .xl_full_scale_set(XlFullScale::_2g)
        .await
        .unwrap();

    /*
     * Configure LIS2MDL target
     */
    let pass = Lsm6dsv16xPassthrough::new_from_sensor(&mut sensor, lis2mdl_addr);
    let mut lis2mdl = lis2mdl::Lis2mdl::new_bus(pass, delay.clone());

    info!("initializing LIS2MDL");

    // Check if LI2MDL connected to Sensor Hub
    match lis2mdl.device_id_get().await {
        Ok(id) => {
            info!("LIS2MDL id: {}", id);
            if id != lis2mdl::ID {
                writeln!(tx, "Device (LIS2MDL) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    lis2mdl.sw_reset().await.unwrap();

    lis2mdl.block_data_update_set(1).await.unwrap();
    lis2mdl.offset_temp_comp_set(1).await.unwrap();
    lis2mdl
        .operating_mode_set(lis2mdl::prelude::Md::ContinuousMode)
        .await.unwrap();
    lis2mdl.data_rate_set(lis2mdl::prelude::Odr::_20hz).await.unwrap();
    /*
     * Configure LPS22DF target
     */

    let pass = Lsm6dsv16xPassthrough::new_from_sensor(&mut sensor, lps22df_addr);
    let mut lps22df = lps22df::Lps22df::new_bus(pass, delay.clone());
    info!("initializing LPS22DF");

    // Check if LPS22DF connected to Sensor Hub.
    match lps22df.id_get().await {
        Ok(id) => {
            info!("LPS22DF id: {}", id);
            if id != lps22df::ID {
                writeln!(tx, "Device (LPS22DF) ID mismatch: {:#02x}", id).unwrap();
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    // Restore default configuration
    lps22df.init_set(lps22df::prelude::Init::Reset).await.unwrap();
    loop {
        let status = lps22df.status_get().await.unwrap();
        if status.sw_reset == 0 {
            break;
        }
    }

    // Set bdu and if if_inc recommeded for driver usage
    lps22df.init_set(lps22df::prelude::Init::DrvRdy).await.unwrap();

    // Select bus interface
    let mut bus_mode = lps22df::prelude::BusMode::default();
    bus_mode.filter = lps22df::prelude::Filter::FilterAuto;
    bus_mode.interface = lps22df::prelude::Interface::SelByHw;
    lps22df.bus_mode_set(&bus_mode).await.unwrap();

    // Set Output Data Rate
    let mut md = lps22df::prelude::Md::default();
    md.odr = lps22df::prelude::Odr::_25hz;
    md.avg = lps22df::prelude::Avg::_16;
    md.lpf = lps22df::prelude::LowPassFilter::OdrDiv4;
    lps22df.mode_set(&md).await.unwrap();

    /*
     *  Slave settings ended: take direct ownership of the master
     */
    // drop to ensure no other borrow_mut from slaves, otherwise borrow_mut()
    // call should be called for every function call of the master

    // Slave instances not more used, take the ownership of the master
    let mut lsm6dsv16x = sensor;
    info!("LSM6DSV16X fifo configuration");

    /*
     * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to FIFO_WATERMARK samples
     */
    lsm6dsv16x.fifo_watermark_set(FIFO_WATERMARK).await.unwrap();

    /* Set FIFO batch XL/Gyro ODR to 60hz */
    lsm6dsv16x.fifo_xl_batch_set(FifoBatch::_60hz).await.unwrap();

    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    lsm6dsv16x.fifo_mode_set(FifoMode::StreamMode).await.unwrap();

    let mut pin_int = PinIntRoute::default();
    pin_int.fifo_th = 1;
    lsm6dsv16x.pin_int1_route_set(&pin_int).await.unwrap();

    // Set Output Data Rate
    lsm6dsv16x.xl_data_rate_set(Odr::_60hz).await.unwrap();
    lsm6dsv16x
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec32)
        .await.unwrap();
    lsm6dsv16x.timestamp_set(1).await.unwrap();

    // Set full scale
    lsm6dsv16x.xl_full_scale_set(XlFullScale::_2g).await.unwrap();

    // Configure filtering chain
    let mut filt_settling_mask = FiltSettlingMask::default();
    filt_settling_mask.drdy = 1;
    filt_settling_mask.irq_xl = 1;
    filt_settling_mask.irq_g = 1;
    lsm6dsv16x
        .filt_settling_mask_set(filt_settling_mask)
        .await.unwrap();
    lsm6dsv16x.filt_xl_lp2_set(1).await.unwrap();
    lsm6dsv16x
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .await.unwrap();

    lsm6dsv16x.xl_data_rate_set(Odr::Off).await.unwrap();

    /*
     * Prepare sensor hub to read data from external slave0 (lis2mdl) and
     * slave1 (lps22df) continuously in order to store data in FIFO.
     */

    let mut sh_cfg_read = ShCfgRead::default();
    sh_cfg_read.slv_add = lis2mdl_addr;
    sh_cfg_read.slv_subadd = lis2mdl::prelude::Reg::OutxLReg as u8;
    sh_cfg_read.slv_len = 6;
    lsm6dsv16x.sh_slv_cfg_read(0, &sh_cfg_read).await.unwrap();
    lsm6dsv16x.fifo_sh_batch_slave_set(0, 1).await.unwrap();

    sh_cfg_read.slv_add = lps22df_addr;
    sh_cfg_read.slv_subadd = lps22df::prelude::Reg::PressOutXl as u8;
    sh_cfg_read.slv_len = 6;
    lsm6dsv16x.sh_slv_cfg_read(1, &sh_cfg_read).await.unwrap();
    lsm6dsv16x.fifo_sh_batch_slave_set(1, 1).await.unwrap();

    // Configure Sensor Hub data rate
    lsm6dsv16x.sh_data_rate_set(ShDataRate::_60hz).await.unwrap();

    // Configure Sensor Hub to read one slave.
    lsm6dsv16x
        .sh_slave_connected_set(ShSlaveConnected::_01)
        .await.unwrap();

    // Set SHUB write_once bit
    lsm6dsv16x
        .sh_write_mode_set(ShWriteMode::OnlyFirstCycle)
        .await.unwrap();

    // Enable I2C Master
    lsm6dsv16x.sh_master_set(1).await.unwrap();

    /* Set Output Data Rate.
     * Selected data rate have to be equal or greater with respect
     * with MLC data rate.
     */
    lsm6dsv16x.xl_data_rate_set(Odr::_120hz).await.unwrap();

    let mut x: i16;
    let mut y: i16;
    let mut z: i16;
    let mut ts: u32;

    info!("sensor configuration eneded, check output on serial device");

    loop {
        int_pin.wait_for_event().await;

        // Read watermark flag
        let fifo_status = lsm6dsv16x.fifo_status_get().await.unwrap();
        let num = fifo_status.fifo_level;

        writeln!(tx, "-- FIFO num {}", num).unwrap();

        for _ in 0..num {
            // Read FIFO sensor value
            let f_data = lsm6dsv16x.fifo_out_raw_get().await.unwrap();
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
