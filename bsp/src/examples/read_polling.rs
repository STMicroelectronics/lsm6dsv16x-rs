use defmt::info;
use maybe_async::maybe_async;
use crate::*;

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

    // Set Output Data Rate for accelerometer and gyroscope
    sensor.xl_data_rate_set(Odr::_7_5hz).await.unwrap();
    sensor.gy_data_rate_set(Odr::_15hz).await.unwrap();

    // Set full scale for accelerometer and gyroscope
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).await.unwrap();

    // Configure filtering chain
    let filt_settling_mask = FiltSettlingMask {
        drdy: 1,
        ois_drdy: 1,
        irq_xl: 1,
        irq_g: 1,
    };
    sensor.filt_settling_mask_set(filt_settling_mask).await.unwrap();
    sensor.filt_gy_lp1_set(1).await.unwrap();
    sensor
        .filt_gy_lp1_bandwidth_set(FiltGyLp1Bandwidth::UltraLight)
        .await
        .unwrap();
    sensor.filt_xl_lp2_set(1).await.unwrap();
    sensor
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .await
        .unwrap();

    info!("Configuration ended, check the output on the UART channel");

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new xl value is available
        let drdy = sensor.flag_data_ready_get().await.unwrap();
        if drdy.drdy_xl == 1 {
            // Read acceleration data
            let data_raw_acceleration = sensor.acceleration_raw_get().await.unwrap();
            let acceleration_mg = [
                from_fs2_to_mg(data_raw_acceleration[0]),
                from_fs2_to_mg(data_raw_acceleration[1]),
                from_fs2_to_mg(data_raw_acceleration[2]),
            ];
            writeln!(
                tx,
                "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]
            ).unwrap();
        }

        // Read output only if new gyroscope value is available
        if drdy.drdy_gy == 1 {
            // Read angular rate data
            let data_raw_angular_rate = sensor.angular_rate_raw_get().await.unwrap();
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
            let data_raw_temperature = sensor.temperature_raw_get().await.unwrap();
            let temperature_deg_c = from_lsb_to_celsius(data_raw_temperature);
            writeln!(tx, "Temperature [degC]: {:.2}", temperature_deg_c).unwrap();
        }

        sensor.tim.delay_ms(1000_u32).await;
    }
}
