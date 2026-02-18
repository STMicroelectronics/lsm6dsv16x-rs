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

    sensor.tilt_mode_set(1).await.unwrap();

    let pin_int = EmbPinIntRoute {
        step_det: 0,
        tilt: 1,
        sig_mot: 0,
        fsm_lc: 0,
    };
    sensor.emb_pin_int1_route_set(pin_int).await.unwrap();

    sensor
        .embedded_int_cfg_set(EmbeddedIntConf::Latched)
        .await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_120hz).await.unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();

    // Wait forever (drdy_xl event handled in IRQ handler)
    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;
        let status = sensor.embedded_status_get().await.unwrap();
        if status.tilt == 1 {
            writeln!(tx, "Tilt event").unwrap();
        }
    }}
