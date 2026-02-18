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

    sensor.xl_data_rate_set(Odr::_15hz).await.unwrap();
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();

    // Configure filtering chain
    let filt_settling_mask = FiltSettlingMask {
        drdy: 1,
        ois_drdy: 0,
        irq_xl: 1,
        irq_g: 1,
    };
    sensor.filt_settling_mask_set(filt_settling_mask).await.unwrap();
    sensor.filt_xl_lp2_set(1).await.unwrap();
    sensor
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .await
        .unwrap();

    // Enable AH/QVAR function
    let mut qvar_mode = AhQvarMode::default();
    qvar_mode.ah_qvar_en = 1;
    sensor.ah_qvar_mode_set(qvar_mode).await.unwrap();

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new values are available
        let all_sources = sensor.all_sources_get().await.unwrap();

        if all_sources.drdy_ah_qvar == 1 {
            let lsb = sensor.ah_qvar_raw_get().await.unwrap();
            let data = from_lsb_to_mv(lsb);

            writeln!(tx, "QVAR [mV]:{:.2}", data).unwrap();
        }
    }
}
