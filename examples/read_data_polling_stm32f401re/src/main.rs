#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;
use cortex_m_rt::entry;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use panic_halt as _;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
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
    let id = sensor.device_id_get().unwrap();
    if id != ID {
        writeln!(tx, "Unexpected device ID: {}", id).unwrap();
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

    // Set Output Data Rate for accelerometer and gyroscope
    sensor.xl_data_rate_set(Odr::_7_5hz).unwrap();
    sensor.gy_data_rate_set(Odr::_15hz).unwrap();

    // Set full scale for accelerometer and gyroscope
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();

    // Configure filtering chain
    let filt_settling_mask = FiltSettlingMask {
        drdy: 1,
        ois_drdy: 1,
        irq_xl: 1,
        irq_g: 1,
    };
    sensor.filt_settling_mask_set(filt_settling_mask).unwrap();
    sensor.filt_gy_lp1_set(1).unwrap();
    sensor
        .filt_gy_lp1_bandwidth_set(FiltGyLp1Bandwidth::UltraLight)
        .unwrap();
    sensor.filt_xl_lp2_set(1).unwrap();
    sensor
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .unwrap();

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new xl value is available
        let drdy = sensor.flag_data_ready_get().unwrap();
        if drdy.drdy_xl == 1 {
            // Read acceleration data
            let data_raw_acceleration = sensor.acceleration_raw_get().unwrap();
            let acceleration_mg = [
                from_fs2_to_mg(data_raw_acceleration[0]),
                from_fs2_to_mg(data_raw_acceleration[1]),
                from_fs2_to_mg(data_raw_acceleration[2]),
            ];
            writeln!(
                tx,
                "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            )
            .unwrap();
        }

        // Read output only if new gyroscope value is available
        if drdy.drdy_gy == 1 {
            // Read angular rate data
            let data_raw_angular_rate = sensor.angular_rate_raw_get().unwrap();
            let angular_rate_mdps = [
                from_fs2000_to_mdps(data_raw_angular_rate[0]),
                from_fs2000_to_mdps(data_raw_angular_rate[1]),
                from_fs2000_to_mdps(data_raw_angular_rate[2]),
            ];
            writeln!(
                tx,
                "Angular rate [mdps]: {:.2}\t{:.2}\t{:.2}",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]
            )
            .unwrap();
        }

        // Read output only if new temperature value is available
        if drdy.drdy_temp == 1 {
            // Read temperature data
            let data_raw_temperature = sensor.temperature_raw_get().unwrap();
            let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
            writeln!(tx, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
        }

        delay.delay_ms(1000_u32);
    }
}
