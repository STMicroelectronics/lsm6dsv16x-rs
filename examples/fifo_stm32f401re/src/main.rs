#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac::{self},
    prelude::*,
    serial::{config::Config, Serial},
};

#[entry]
fn main() -> ! {
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
    // Enable Block Data Update
    sensor.block_data_update_set(1).unwrap();

    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();
    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to FIFO_WATERMARK samples
    sensor.fifo_watermark_set(25).unwrap();

    // Set FIFO batch XL/Gyro ODR to 12.5hz
    sensor.fifo_xl_batch_set(FifoBatch::_60hz).unwrap();
    sensor.fifo_gy_batch_set(FifoBatch::_15hz).unwrap();

    // Set FIFO mode to Stream mode
    sensor.fifo_mode_set(FifoMode::StreamMode).unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_60hz).unwrap();
    sensor.gy_data_rate_set(Odr::_15hz).unwrap();

    sensor
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec8)
        .unwrap();
    sensor.timestamp_set(1).unwrap();

    loop {
        let status: AllSources = sensor.all_sources_get().unwrap();
        let fifo_status: FifoStatus = sensor.fifo_status_get().unwrap();
        let num = fifo_status.fifo_level;
        if status.fifo_th == 1 {
            writeln!(tx, "-- FIFO num {}\r\n", num).unwrap();

            for _ in 0..num {
                let f_data: FifoOutRaw = sensor.fifo_out_raw_get().unwrap();
                let datax = i16::from_le_bytes([f_data.data[0], f_data.data[1]]);
                let datay = i16::from_le_bytes([f_data.data[2], f_data.data[3]]);
                let dataz = i16::from_le_bytes([f_data.data[4], f_data.data[5]]);
                let ts = u32::from_le_bytes([
                    f_data.data[0],
                    f_data.data[1],
                    f_data.data[2],
                    f_data.data[3],
                ]);

                match f_data.tag {
                    Tag::XlNcTag => {
                        writeln!(
                            tx,
                            "ACC [mg]:\t{:.2}\t{:.2}\t{:.2}\r\n",
                            from_fs2_to_mg(datax),
                            from_fs2_to_mg(datay),
                            from_fs2_to_mg(dataz)
                        )
                        .unwrap();
                    }
                    Tag::GyNcTag => {
                        writeln!(
                            tx,
                            "GYR [mdps]:\t{:.2}\t{:.2}\t{:.2}\r\n",
                            from_fs2000_to_mdps(datax),
                            from_fs2000_to_mdps(datay),
                            from_fs2000_to_mdps(dataz)
                        )
                        .unwrap();
                    }
                    Tag::TimestampTag => {
                        let ts_usec = from_lsb_to_nsec(ts) / 1000.0;
                        writeln!(tx, "TIMESTAMP {:.1} [us] (lsb: {})\r\n", ts_usec, ts).unwrap();
                    }
                    _ => {}
                }
            }

            writeln!(tx, "------ \r\n\r\n").unwrap();
        }
    }
}
