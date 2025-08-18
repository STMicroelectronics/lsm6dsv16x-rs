#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use st_fifo_tool;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

static FIFO_WATERMARK: u8 = 32;
static FIFO_COMPRESSION: u8 = 3;
static SLOT_NUMBER: u8 = FIFO_WATERMARK * FIFO_COMPRESSION;

#[entry]
fn main() -> ! {
    // Define private variables
    let mut raw_slot = [st_fifo_tool::RawSlot::default(); SLOT_NUMBER as usize];
    let mut out_slot = [st_fifo_tool::OutSlot::default(); SLOT_NUMBER as usize];
    let mut acc_slot = [st_fifo_tool::OutSlot::default(); SLOT_NUMBER as usize];
    let mut gyr_slot = [st_fifo_tool::OutSlot::default(); SLOT_NUMBER as usize];

    let dp = pac::Peripherals::take().unwrap();
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

    delay.delay_ms(5);

    let mut sensor = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, tim1);

    // Check device ID
    let whoami = sensor.device_id_get().unwrap();
    if whoami != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(Reset::RestoreCtrlRegs).unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().unwrap();
    }

    let fifo_config = st_fifo_tool::Config {
        device: st_fifo_tool::DeviceType::Lsm6dsv16x,
        bdr_xl: 15.0,
        bdr_gy: 15.0,
        bdr_vsens: 0.0,
    };

    let mut fifo = st_fifo_tool::FifoData::init(&fifo_config).unwrap();

    // Enable Block Data Update
    sensor.block_data_update_set(1).unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();
    /*
     * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to FIFO_WATERMARK samples
     */
    sensor.fifo_watermark_set(FIFO_WATERMARK).unwrap();
    /* Set FIFO batch XL/Gyro ODR to 15hz */
    sensor.fifo_xl_batch_set(FifoBatch::_15hz).unwrap();
    sensor.fifo_gy_batch_set(FifoBatch::_15hz).unwrap();
    /* Set FIFO mode to Stream mode (aka Continuous Mode) */
    sensor.fifo_mode_set(FifoMode::StreamMode).unwrap();
    /* Enable FIFO compression on all samples */
    sensor
        .fifo_compress_algo_set(FifoCompressAlgo::_8To1)
        .unwrap();
    sensor.fifo_compress_algo_real_time_set(1).unwrap();

    /* Set Output Data Rate */
    sensor.xl_data_rate_set(Odr::_15hz).unwrap();
    sensor.gy_data_rate_set(Odr::_15hz).unwrap();
    sensor
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec8)
        .unwrap();
    sensor.timestamp_set(1).unwrap();

    let mut slots: u16;
    let mut acc_samples;
    let mut gyr_samples;
    loop {
        let mut out_slot_size: u16 = 0;

        let fifo_status = sensor.fifo_status_get().unwrap();
        slots = 0;

        if fifo_status.fifo_th == 1 {
            let num = if fifo_status.fifo_level <= SLOT_NUMBER as u16 {
                fifo_status.fifo_level
            } else {
                SLOT_NUMBER as u16
            };

            writeln!(tx, "-- FIFO num {}", num).unwrap();

            for _ in 0..num {
                // Read FIFO sensor value
                let f_data = sensor.fifo_out_raw_get().unwrap();
                raw_slot[slots as usize].fifo_data_out[0] =
                    ((f_data.tag as u8) << 3) | (f_data.cnt << 1);
                raw_slot[slots as usize].fifo_data_out[1..].copy_from_slice(&f_data.data);

                slots += 1;
            }

            // Uncompress FIFO samples and filter based on sensor type
            fifo.decode(&mut out_slot, &raw_slot, &mut out_slot_size, slots);
            fifo.sort(&mut out_slot, out_slot_size);

            // Count how many acc and gyro samples
            acc_samples = fifo.get_sensor_occurrence(
                &out_slot,
                out_slot_size,
                st_fifo_tool::SensorType::Accelerometer,
            );
            gyr_samples = fifo.get_sensor_occurrence(
                &out_slot,
                out_slot_size,
                st_fifo_tool::SensorType::Gyroscope,
            );

            // Extract acc and gyro samples
            fifo.extract_sensor(
                &mut acc_slot,
                &out_slot,
                out_slot_size,
                st_fifo_tool::SensorType::Accelerometer,
            );
            fifo.extract_sensor(
                &mut gyr_slot,
                &out_slot,
                out_slot_size,
                st_fifo_tool::SensorType::Gyroscope,
            );

            for i in 0..acc_samples as usize {
                let entry = acc_slot[i];
                let axis = entry.sensor_data.to_axis();

                let x = from_fs2_to_mg(axis.x);
                let y = from_fs2_to_mg(axis.y);
                let z = from_fs2_to_mg(axis.z);
                writeln!(
                    tx,
                    "ACC:\t{}\t{}\t{:.2}\t{:.2}\t{:.2}",
                    entry.timestamp, entry.sensor_tag, x, y, z
                )
                .unwrap();
            }

            for i in 0..gyr_samples as usize {
                let entry = &gyr_slot[i];
                let axis = entry.sensor_data.to_axis();

                let x = from_fs2000_to_mdps(axis.x);
                let y = from_fs2000_to_mdps(axis.y);
                let z = from_fs2000_to_mdps(axis.z);
                writeln!(
                    tx,
                    "GYR:\t{}\t{}\t{:.2}\t{:.2}\t{:.2}",
                    entry.timestamp, entry.sensor_tag, x, y, z
                )
                .unwrap();
            }
        }
    }
}
