#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, delay.clone());

    // Check device ID
    let id = sensor.device_id_get().unwrap();
    if id != ID {
        writeln!(&mut msg, "Unexpected device ID: {}", id).unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
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
                &mut msg,
                "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            )
            .unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
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
                &mut msg,
                "Angular rate [mdps]: {:.2}\t{:.2}\t{:.2}",
                angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]
            )
            .unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }

        // Read output only if new temperature value is available
        if drdy.drdy_temp == 1 {
            // Read temperature data
            let data_raw_temperature = sensor.temperature_raw_get().unwrap();
            let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
            writeln!(&mut msg, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }

        delay.delay_ms(1000_u32);
    }
}
