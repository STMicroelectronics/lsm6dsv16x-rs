#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use libm::sqrtf;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;

const FIFO_WATERMARK: u8 = 32;

fn sflp2q(sflp: &[u16; 3]) -> [f32; 4] {
    let mut sumsq = 0f32;
    let mut quat = [0f32; 4];

    for i in 0..3 {
        quat[i] = from_half_to_single_precision(sflp[i]) as f32;
    }

    for i in 0..3 {
        sumsq += quat[i] * quat[i];
    }

    if sumsq > 1.0 {
        let n = sqrtf(sumsq);
        for i in 0..3 {
            quat[i] /= n;
        }
        sumsq = 1.0f32;
    }

    quat[3] = sqrtf(1.0f32 - sumsq);

    quat
}

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    delay.delay_ms(5_u32);

    let mut msg: String<256> = String::new();

    let mut sensor = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, delay.clone());

    // Check device ID
    let whoami = sensor.device_id_get().unwrap();
    if whoami != ID {
        writeln!(&mut msg, "Device ID mismatch: {:#02x}", whoami).unwrap();
        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
        loop {}
    }

    // Restore default configuration
    sensor.reset_set(Reset::RestoreCtrlRegs).unwrap();
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = sensor.reset_get().unwrap();
    }
    // Enable Block Data Update
    sensor.block_data_update_set(1).unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_4g).unwrap();
    sensor.gy_full_scale_set(GyFullScale::_2000dps).unwrap();

    /*
     * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
     * stored in FIFO) to FIFO_WATERMARK samples
     */
    sensor.fifo_watermark_set(FIFO_WATERMARK).unwrap();

    // Set FIFO batch of sflp data
    let mut fifo_sflp = FifoSflpRaw::default();
    fifo_sflp.game_rotation = 1;
    fifo_sflp.gravity = 1;
    fifo_sflp.gbias = 1;
    sensor.fifo_sflp_batch_set(fifo_sflp).unwrap();

    // Set FIFO mode to Stream mode (aka Continuous Mode)
    sensor.fifo_mode_set(FifoMode::StreamMode).unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_30hz).unwrap();
    sensor.gy_data_rate_set(Odr::_30hz).unwrap();
    sensor.sflp_data_rate_set(SflpDataRate::_30hz).unwrap();

    sensor.sflp_game_rotation_set(1).unwrap();

    /*
     * here application may initialize offset with latest values
     * calculated from previous run and saved to non volatile memory.
     */
    let mut gbias = SflpGbias::default();
    gbias.gbias_x = 0.0f32;
    gbias.gbias_y = 0.0f32;
    gbias.gbias_z = 0.0f32;
    sensor.sflp_game_gbias_set(&gbias).unwrap();

    // Wait samples
    let mut gravity_mg = [0f32; 3];
    let mut gbias_mdps = [0f32; 3];
    loop {
        let fifo_status = sensor.fifo_status_get().unwrap();

        if fifo_status.fifo_th == 1 {
            let num = fifo_status.fifo_level;

            writeln!(&mut msg, "-- FIFO num {}", num).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();

            for _ in 0..num {
                let raw_data = sensor.fifo_out_raw_get().unwrap();
                let axis = raw_data.data;

                let axis_3d: [u16; 3] = [
                    (axis[0] as u16) | ((axis[1] as u16) << 8),
                    (axis[2] as u16) | ((axis[3] as u16) << 8),
                    (axis[4] as u16) | ((axis[5] as u16) << 8),
                ];

                match raw_data.tag {
                    Tag::SflpGyroscopeBiasTag => {
                        for i in 0..3 {
                            gbias_mdps[i] = from_fs125_to_mdps(axis_3d[i] as i16);
                        }

                        writeln!(
                            &mut msg,
                            "GBIAS [mdps]:{:.2}\t{:.2}\t{:.2}",
                            gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]
                        )
                        .unwrap();
                        let _ = tx.blocking_write(msg.as_bytes());
                        msg.clear();
                    }
                    Tag::SflpGravityVectorTag => {
                        for i in 0..3 {
                            gravity_mg[i] = from_sflp_to_mg(axis_3d[i] as i16);
                        }

                        writeln!(
                            &mut msg,
                            "Gravity [mg]:{:.2}\t{:.2}\t{:.2}",
                            gravity_mg[0], gravity_mg[1], gravity_mg[2]
                        )
                        .unwrap();
                        let _ = tx.blocking_write(msg.as_bytes());
                        msg.clear();
                    }
                    Tag::SflpGameRotationVectorTag => {
                        let quat = sflp2q(&axis_3d);
                        writeln!(&mut msg, "[{:x} {:x} {:x} {:x} {:x} {:x}] Game Rotation \tX: {:.3}\tY{:.3}\tZ{:.3}\tW{:.3}", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], quat[0], quat[1], quat[2], quat[3]).unwrap();
                        let _ = tx.blocking_write(msg.as_bytes());
                        msg.clear();
                    }
                    _ => {}
                }
            }
        }
    }
}
