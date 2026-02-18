use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, delay: D, mut int_pin: I) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
    I: InterruptPin
{
    use lsm6dsv16x::prelude::*;

    info!("Configuring the sensor");
    let mut sensor = Lsm6dsv16x::from_bus(bus, delay);

    // Check device ID
    let whoami = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", whoami);
    if whoami != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
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

    let mut pin_int = PinIntRoute::default();
    pin_int.drdy_xl = 1;
    sensor.pin_int1_route_set(&pin_int).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_120hz).await.unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();
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

    // Wait forever (drdy_xl event handled in IRQ handler)
    loop {
        int_pin.wait_for_event().await;
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
            )
            .unwrap();
        }
    }
}
