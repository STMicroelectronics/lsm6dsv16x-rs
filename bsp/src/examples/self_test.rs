use defmt::info;
use maybe_async::maybe_async;
use core::fmt;
use crate::*;
use libm::fabsf;

enum StResult {
    StPass = 1,
    StFail = 0,
}

impl fmt::Display for StResult {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            StResult::StPass => core::write!(f, "Pass"),
            StResult::StFail => core::write!(f, "Fail"),
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
            StTestType::StPos => core::write!(f, "POS"),
            StTestType::StNeg => core::write!(f, "NEG"),
        }
    }
}

const MIN_ST_LIMIT_MG: f32 = 50.0;
const MAX_ST_LIMIT_MG: f32 = 1700.0;
const MIN_ST_LIMIT_MDPS: f32 = 150000.0;
const MAX_ST_LIMIT_MDPS: f32 = 700000.0;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, delay: D, _irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use lsm6dsv16x::prelude::*;

    info!("Configuring the sensor");
    let mut sensor = Lsm6dsv16x::from_bus(bus, delay);

    // boot time
    sensor.tim.delay_ms(5).await;

    // Check device ID
    let id = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(Reset::RestoreCtrlRegs).await.unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().await.unwrap();
    }

    // Enable Block Data Update
    sensor.block_data_update_set(1).await.unwrap();

    let mut val_st_off: [f32; 3];
    let mut val_st_on: [f32; 3];
    let mut test_val = [0.0f32; 3];

    for test in [StTestType::StPos, StTestType::StNeg] {
        /*
         * Accelerometer Self Test
         */

        // Set Output Data Rate
        sensor.xl_data_rate_set(Odr::_60hz).await.unwrap();
        // Set full scale
        sensor.xl_full_scale_set(XlFullScale::_4g).await.unwrap();
        // Wait stable output
        sensor.tim.delay_ms(100).await;

        // Check if new value available
        let mut drdy_xl = 0;
        while drdy_xl == 0 {
            drdy_xl = sensor.flag_data_ready_get().await.unwrap().drdy_xl;
        }

        // Read dummy data and discard it
        let _dummy = sensor.acceleration_raw_get().await.unwrap();

        // Read 5 samples and get the average value for each axis
        val_st_off = [0.0f32; 3];
        for _ in 0..5 {
            let mut drdy = sensor.flag_data_ready_get().await.unwrap();
            while drdy.drdy_xl == 0 {
                drdy = sensor.flag_data_ready_get().await.unwrap();
            }

            let data_raw = sensor.acceleration_raw_get().await.unwrap();
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
        sensor.xl_self_test_set(type_set).await.unwrap();
        // Wait stable output
        sensor.tim.delay_ms(100).await;

        // Check if new value available
        drdy_xl = 0;
        while drdy_xl == 0 {
            drdy_xl = sensor.flag_data_ready_get().await.unwrap().drdy_xl;
        }

        // Read dummy data and discard it
        let _dummy = sensor.acceleration_raw_get().await.unwrap();
        val_st_on = [0.0f32; 3];

        // Read 5 samples and get the average value for each axis
        for _ in 0..5 {
            // Check if new value available
            drdy_xl = 0;
            while drdy_xl == 0 {
                drdy_xl = sensor.flag_data_ready_get().await.unwrap().drdy_xl;
            }

            // Read data and accumulate the mg value
            let data_raw = sensor.acceleration_raw_get().await.unwrap();
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
        sensor.xl_self_test_set(XlSelfTest::Disable).await.unwrap();
        // Disable sensor
        sensor.xl_data_rate_set(Odr::Off).await.unwrap();

        /*
         * Gyroscope Self Test
         */
        // Set Output Data Rate
        sensor.gy_data_rate_set(Odr::_240hz).await.unwrap();
        // Set full scale
        sensor.gy_full_scale_set(GyFullScale::_2000dps).await.unwrap();
        // Wait stable output
        sensor.tim.delay_ms(100).await;

        val_st_off = [0.0f32; 3];
        for _ in 0..5 {
            // Check if new value available
            let mut drdy_gy = 0;
            while drdy_gy == 0 {
                drdy_gy = sensor.flag_data_ready_get().await.unwrap().drdy_gy;
            }
            // Read and accumulate the mg value
            let data_raw = sensor.angular_rate_raw_get().await.unwrap();
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
        sensor.gy_self_test_set(type_set).await.unwrap();
        sensor.tim.delay_ms(100).await;

        // Read 5 samples and get the average value for each axis
        val_st_on = [0.0f32; 3];
        for _ in 0..5 {
            // Check if new value available
            let mut drdy = sensor.flag_data_ready_get().await.unwrap();
            while drdy.drdy_gy == 0 {
                drdy = sensor.flag_data_ready_get().await.unwrap();
            }

            // Read data and accumulate the mg value
            let data_raw = sensor.angular_rate_raw_get().await.unwrap();
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
        sensor.gy_self_test_set(GySelfTest::Disable).await.unwrap();
        sensor.gy_data_rate_set(Odr::Off).await.unwrap();

        // Output the result
        writeln!(tx, "Self Test {} - {}", test, st_result).unwrap();
    }

    loop {}
}
