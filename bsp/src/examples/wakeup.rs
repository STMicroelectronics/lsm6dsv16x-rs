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

    let mut pin_int = PinIntRoute::default();
    pin_int.wakeup = 1;
    sensor.pin_int1_route_set(&pin_int).await.unwrap();

    let irq = InterruptMode { enable: 1, lir: 1 };
    sensor.interrupt_enable_set(irq).await.unwrap();

    sensor.filt_xl_fast_settling_set(1).await.unwrap();
    sensor.mask_trigger_xl_settl_set(1).await.unwrap();
    sensor
        .filt_wkup_act_feed_set(FiltWkupActFeed::HighPass)
        .await.unwrap();

    let mut inact_cfg = InactivityDur::new();
    inact_cfg.set_inact_dur(0);
    inact_cfg.set_xl_inact_odr(1);
    inact_cfg.set_wu_inact_ths_w(3); // threshold resolution to 62.5 mg
    let threshold = ActThresholds {
        inactivity_cfg: inact_cfg,
        inactivity_ths: 0,
        threshold: 1,
        duration: 0,
    };

    sensor.act_thresholds_set(threshold).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_480hz).await.unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();

    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;

        let status = sensor.all_sources_get().await.unwrap();
        if status.wake_up == 1 {
            writeln!(tx, "Wakeup event").unwrap();
        }
    }
}
