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
    pin_int.single_tap = 1;
    pin_int.double_tap = 1;
    sensor.pin_int1_route_set(&pin_int).await.unwrap();

    // Enable interrupts
    let irq: InterruptMode = InterruptMode { enable: 1, lir: 0 };
    sensor.interrupt_enable_set(irq).await.unwrap();

    let tap = TapDetection {
        tap_x_en: 0,
        tap_y_en: 0,
        tap_z_en: 1,
    };
    sensor.tap_detection_set(tap).await.unwrap();

    // Set wake-up thresholds and duration
    let thr = TapThresholds { x: 0, y: 0, z: 3 };
    sensor.tap_thresholds_set(thr).await.unwrap();

    let tap_win = TapTimeWindows {
        shock: 3,
        quiet: 3,
        tap_gap: 7,
    };
    sensor.tap_time_windows_set(tap_win).await.unwrap();
    sensor.tap_mode_set(TapMode::BothSingleDouble).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_480hz).await.unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_8g).await.unwrap();

    // Wait forever (drdy_xl event handled in IRQ handler)
    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;

        let status = sensor.all_sources_get().await.unwrap();
        if status.single_tap == 1 {
            writeln!(tx, "Single TAP").unwrap();
        }
        if status.double_tap == 1 {
            writeln!(tx, "Double TAP").unwrap();
        }
    }
}
