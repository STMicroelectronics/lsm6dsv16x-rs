#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::{self, Write};
use cortex_m_rt::entry;
use libm::fabsf;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use panic_halt as _;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

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
        delay.delay_ms(100);

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
        delay.delay_ms(100);

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
        delay.delay_ms(100);

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
        delay.delay_ms(100);

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
        writeln!(tx, "Self Test {} - {}", test, st_result).unwrap();
    }

    loop {}
}
