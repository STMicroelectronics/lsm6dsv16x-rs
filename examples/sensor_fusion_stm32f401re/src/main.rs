#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;
use cortex_m_rt::entry;
use libm::sqrtf;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();

    delay.delay_ms(5);

    let mut sensor = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, tim1);

    // Check device ID
    let whoami = sensor.device_id_get().unwrap();
    if whoami != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
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

            writeln!(tx, "-- FIFO num {}", num).unwrap();

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
                            tx,
                            "GBIAS [mdps]:{:.2}\t{:.2}\t{:.2}",
                            gbias_mdps[0], gbias_mdps[1], gbias_mdps[2]
                        )
                        .unwrap();
                    }
                    Tag::SflpGravityVectorTag => {
                        for i in 0..3 {
                            gravity_mg[i] = from_sflp_to_mg(axis_3d[i] as i16);
                        }

                        writeln!(
                            tx,
                            "Gravity [mg]:{:.2}\t{:.2}\t{:.2}",
                            gravity_mg[0], gravity_mg[1], gravity_mg[2]
                        )
                        .unwrap();
                    }
                    Tag::SflpGameRotationVectorTag => {
                        let quat = sflp2q(&axis_3d);
                        writeln!(tx, "[{:x} {:x} {:x} {:x} {:x} {:x}] Game Rotation \tX: {:.3}\tY{:.3}\tZ{:.3}\tW{:.3}", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], quat[0], quat[1], quat[2], quat[3]).unwrap();
                    }
                    _ => {}
                }
            }
        }
    }
}
