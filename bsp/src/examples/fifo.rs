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

    sensor.reset_set(Reset::RestoreCtrlRegs).await.unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().await.unwrap();
    }
    // Enable Block Data Update
    sensor.block_data_update_set(1).await.unwrap();

    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).await.unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).await.unwrap();
    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to FIFO_WATERMARK samples
    sensor.fifo_watermark_set(25).await.unwrap();

    // Set FIFO batch XL/Gyro ODR to 12.5hz
    sensor.fifo_xl_batch_set(FifoBatch::_60hz).await.unwrap();
    sensor.fifo_gy_batch_set(FifoBatch::_15hz).await.unwrap();

    // Set FIFO mode to Stream mode
    sensor.fifo_mode_set(FifoMode::StreamMode).await.unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_60hz).await.unwrap();
    sensor.gy_data_rate_set(Odr::_15hz).await.unwrap();

    sensor
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec8)
        .await
        .unwrap();
    sensor.timestamp_set(1).await.unwrap();

    loop {
        let status: AllSources = sensor.all_sources_get().await.unwrap();
        let fifo_status: FifoStatus = sensor.fifo_status_get().await.unwrap();
        let num = fifo_status.fifo_level;
        if status.fifo_th == 1 {
            writeln!(tx, "-- FIFO num {}\r\n", num).unwrap();

            for _ in 0..num {
                let f_data: FifoOutRaw = sensor.fifo_out_raw_get().await.unwrap();
                let datax = i16::from_le_bytes([f_data.data[0], f_data.data[1]]);
                let datay = i16::from_le_bytes([f_data.data[2], f_data.data[3]]);
                let dataz = i16::from_le_bytes([f_data.data[4], f_data.data[5]]);
                let ts = u32::from_le_bytes([
                    f_data.data[0],
                    f_data.data[1],
                    f_data.data[2],
                    f_data.data[3],
                ]);

                match f_data.tag {
                    Tag::XlNcTag => {
                        writeln!(
                            tx,
                            "ACC [mg]:\t{:.2}\t{:.2}\t{:.2}\r\n",
                            from_fs2_to_mg(datax),
                            from_fs2_to_mg(datay),
                            from_fs2_to_mg(dataz)
                        )
                        .unwrap();
                    }
                    Tag::GyNcTag => {
                        writeln!(
                            tx,
                            "GYR [mdps]:\t{:.2}\t{:.2}\t{:.2}\r\n",
                            from_fs2000_to_mdps(datax),
                            from_fs2000_to_mdps(datay),
                            from_fs2000_to_mdps(dataz)
                        )
                        .unwrap();
                    }
                    Tag::TimestampTag => {
                        let ts_usec = from_lsb_to_nsec(ts) / 1000.0;
                        writeln!(tx, "TIMESTAMP {:.1} [us] (lsb: {})\r\n", ts_usec, ts).unwrap();
                    }
                    _ => {}
                }
            }

            writeln!(tx, "------ \r\n\r\n").unwrap();
        }
    }
}
