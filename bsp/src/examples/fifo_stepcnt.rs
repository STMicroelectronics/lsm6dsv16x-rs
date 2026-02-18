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

    sensor.tim.delay_ms(5).await;

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
    pin_int.fifo_th = 1;
    sensor.pin_int1_route_set(&pin_int).await.unwrap();

    // // Enable interrupts
    let irq: InterruptMode = InterruptMode {
         enable: 1,
         lir: 1,
    };
    sensor.interrupt_enable_set(irq).await.unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();
    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to FIFO_WATERMARK samples
    sensor.fifo_watermark_set(25).await.unwrap();

    // Enable pedometer
    let stpcnt_mode: StpcntMode = StpcntMode {
        step_counter_enable: 1,
        false_step_rej: 1,
    };
    sensor.stpcnt_mode_set(stpcnt_mode).await.unwrap();
    // Enable step counter data in FIFO
    sensor.fifo_stpcnt_batch_set(1).await.unwrap();

    // Set FIFO mode to Stream mode
    sensor.fifo_mode_set(FifoMode::StreamMode).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_60hz).await.unwrap();

    sensor
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec32)
        .await
        .unwrap();
    sensor.timestamp_set(1).await.unwrap();

    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;
        let status: AllSources = sensor.all_sources_get().await.unwrap();
        let fifo_status: FifoStatus = sensor.fifo_status_get().await.unwrap();
        let num = fifo_status.fifo_level;
        if status.fifo_th == 1 {
            writeln!(tx, "-- FIFO num {}\r\n", num).unwrap();

            for _ in 0..num {
                let f_data: FifoOutRaw = sensor.fifo_out_raw_get().await.unwrap();
                let steps = i16::from_le_bytes([f_data.data[0], f_data.data[1]]);
                let ts = u32::from_le_bytes([
                    f_data.data[2],
                    f_data.data[3],
                    f_data.data[4],
                    f_data.data[5],
                ]);

                writeln!(tx, "out {}\r\n", f_data.tag as u8).unwrap();
                match f_data.tag {
                    Tag::StepCounterTag => {
                        writeln!(
                            tx,
                            "Steps:\t{:.2}\t{:.2}us\t\r\n",
                            steps,
                            from_lsb_to_nsec(ts) / 1000.0,
                        )
                        .unwrap();
                    }
                    _ => {}
                }
            }

            writeln!(tx, "------ \r\n\r\n").unwrap();
        }
    }
}
