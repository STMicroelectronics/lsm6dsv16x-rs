use defmt::info;
use maybe_async::maybe_async;
use crate::*;

use crate::config::fsm_config::FOUR_D;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

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

    for ucf_entry in FOUR_D {
        match ucf_entry.op {
            MemsUcfOp::Delay => {
                sensor.tim.delay_ms(ucf_entry.data.into()).await;
            }
            MemsUcfOp::Write => {
                sensor
                    .bus
                    .write_to_register(ucf_entry.address as u8, &[ucf_entry.data])
                    .await
                    .unwrap();
            }
            _ => {}
        }
    }

    loop {
        int_pin.wait_for_event().await;
        let status = sensor.all_sources_get().await.unwrap();
        if status.fsm1 == 1 {
            let fsm_out = sensor.fsm_out_get().await.unwrap();
            match fsm_out[0].into_bits() {
                0x10 => {
                    writeln!(tx, "Y down event").unwrap();
                }
                0x20 => {
                    writeln!(tx, "Y up event").unwrap();
                }
                0x40 => {
                    writeln!(tx, "X down event").unwrap();
                }
                0x80 => {
                    writeln!(tx, "X up event").unwrap();
                }
                _ => {}
            }
        }
    }
}
