#![no_std]
#![no_main]

use core::fmt::{self, Write};
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
use libm::fabsf;
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

enum StResult {
    StPass = 1,
    StFail = 0,
}

impl fmt::Display for StResult {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            StResult::StPass => write!(f, "Pass"),
            StResult::StFail => write!(f, "Fail"),
        }
    }
}

#[repr(u8)]
#[derive(PartialEq, Debug)]
enum StTestType {
    StPos = 0,
    StNeg = 1,
}

impl fmt::Display for StTestType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            StTestType::StPos => write!(f, "POS"),
            StTestType::StNeg => write!(f, "NEG"),
        }
    }
}

const MIN_ST_LIMIT_MG: f32 = 50.0;
const MAX_ST_LIMIT_MG: f32 = 1700.0;
const MIN_ST_LIMIT_MDPS: f32 = 150000.0;
const MAX_ST_LIMIT_MDPS: f32 = 700000.0;

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

    let mut val_st_off: [f32; 3];
    let mut val_st_on: [f32; 3];
    let mut test_val = [0.0f32; 3];

    for test in [StTestType::StPos, StTestType::StNeg] {
        /*
         * Accelerometer Self Test
         */

        // Set Output Data Rate
        sensor.xl_data_rate_set(Odr::_60hz).unwrap();
        // Set full scale
        sensor.xl_full_scale_set(XlFullScale::_4g).unwrap();
        // Wait stable output
        delay.delay_ms(100_u32);

        // Check if new value available
        let mut drdy_xl = 0;
        while drdy_xl == 0 {
            drdy_xl = sensor.flag_data_ready_get().unwrap().drdy_xl;
        }

        // Read dummy data and discard it
        let _dummy = sensor.acceleration_raw_get().unwrap();

        // Read 5 samples and get the average value for each axis
        val_st_off = [0.0f32; 3];
        for _ in 0..5 {
            let mut drdy = sensor.flag_data_ready_get().unwrap();
            while drdy.drdy_xl == 0 {
                drdy = sensor.flag_data_ready_get().unwrap();
            }

            let data_raw = sensor.acceleration_raw_get().unwrap();
            for j in 0..3 {
                val_st_off[j] += from_fs4_to_mg(data_raw[j]);
            }
        }

        // Calculate the mg average values
        for i in 0..3 {
            val_st_off[i] /= 5.0;
        }

        // Enable Self Test positive (or negative)
        let type_set = if test == StTestType::StPos {
            XlSelfTest::Positive
        } else {
            XlSelfTest::Negative
        };
        sensor.xl_self_test_set(type_set).unwrap();
        // Wait stable output
        delay.delay_ms(100_u32);

        // Check if new value available
        drdy_xl = 0;
        while drdy_xl == 0 {
            drdy_xl = sensor.flag_data_ready_get().unwrap().drdy_xl;
        }

        // Read dummy data and discard it
        let _dummy = sensor.acceleration_raw_get().unwrap();
        val_st_on = [0.0f32; 3];

        // Read 5 samples and get the average value for each axis
        for _ in 0..5 {
            // Check if new value available
            drdy_xl = 0;
            while drdy_xl == 0 {
                drdy_xl = sensor.flag_data_ready_get().unwrap().drdy_xl;
            }

            // Read data and accumulate the mg value
            let data_raw = sensor.acceleration_raw_get().unwrap();
            for j in 0..3 {
                val_st_on[j] += from_fs4_to_mg(data_raw[j]);
            }
        }

        // Calculate the mg average values
        for i in 0..3 {
            val_st_on[i] /= 5.0;
        }

        // Calculate the mg values for self test
        for i in 0..3 {
            test_val[i] = fabsf(val_st_on[i] - val_st_off[i]);
        }

        // Check self test limit
        let mut st_result = StResult::StPass;
        for i in 0..3 {
            if (MIN_ST_LIMIT_MG > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_MG) {
                st_result = StResult::StFail;
            }
        }

        // Disable Self Test
        sensor.xl_self_test_set(XlSelfTest::Disable).unwrap();
        // Disable sensor
        sensor.xl_data_rate_set(Odr::Off).unwrap();

        /*
         * Gyroscope Self Test
         */
        // Set Output Data Rate
        sensor.gy_data_rate_set(Odr::_240hz).unwrap();
        // Set full scale
        sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();
        // Wait stable output
        delay.delay_ms(100_u32);

        val_st_off = [0.0f32; 3];
        for _ in 0..5 {
            // Check if new value available
            let mut drdy_gy = 0;
            while drdy_gy == 0 {
                drdy_gy = sensor.flag_data_ready_get().unwrap().drdy_gy;
            }
            // Read and accumulate the mg value
            let data_raw = sensor.angular_rate_raw_get().unwrap();
            for j in 0..3 {
                val_st_off[j] += from_fs2000_to_mdps(data_raw[j]);
            }
        }

        for i in 0..3 {
            val_st_off[i] /= 5.0;
        }

        // Enable Self Test positive
        let type_set = if test == StTestType::StPos {
            GySelfTest::Positive
        } else {
            GySelfTest::Negative
        };
        sensor.gy_self_test_set(type_set).unwrap();
        delay.delay_ms(100_u32);

        // Read 5 samples and get the average value for each axis
        val_st_on = [0.0f32; 3];
        for _ in 0..5 {
            // Check if new value available
            let mut drdy = sensor.flag_data_ready_get().unwrap();
            while drdy.drdy_gy == 0 {
                drdy = sensor.flag_data_ready_get().unwrap();
            }

            // Read data and accumulate the mg value
            let data_raw = sensor.angular_rate_raw_get().unwrap();
            for j in 0..3 {
                val_st_on[j] += from_fs2000_to_mdps(data_raw[j]);
            }
        }

        // Calculate the mg average values
        for i in 0..3 {
            val_st_on[i] /= 5.0;
        }

        // Calculate the mg values for self test
        for i in 0..3 {
            test_val[i] = fabsf(val_st_on[i] - val_st_off[i]);
        }

        // Check self test limit
        for i in 0..3 {
            if (MIN_ST_LIMIT_MDPS > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_MDPS) {
                st_result = StResult::StFail;
            }
        }

        // Disable Self Test
        sensor.gy_self_test_set(GySelfTest::Disable).unwrap();
        sensor.gy_data_rate_set(Odr::Off).unwrap();

        // Output the result
        writeln!(&mut msg, "Self Test {} - {}", test, st_result).unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
    }

    loop {}
}
