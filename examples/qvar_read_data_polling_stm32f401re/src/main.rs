#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;
use cortex_m_rt::entry;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use panic_halt as _;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::{config::Config, Serial},
};

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
    let id = sensor.device_id_get().unwrap();
    if id != ID {
        writeln!(tx, "Unexpected device ID: {}", id).unwrap();
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

    sensor.xl_data_rate_set(Odr::_15hz).unwrap();
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();

    // Configure filtering chain
    let filt_settling_mask = FiltSettlingMask {
        drdy: 1,
        ois_drdy: 0,
        irq_xl: 1,
        irq_g: 1,
    };
    sensor.filt_settling_mask_set(filt_settling_mask).unwrap();
    sensor.filt_xl_lp2_set(1).unwrap();
    sensor
        .filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .unwrap();

    // Enable AH/QVAR function
    let mut qvar_mode = AhQvarMode::default();
    qvar_mode.ah_qvar_en = 1;
    sensor.ah_qvar_mode_set(qvar_mode).unwrap();

    // Read samples in polling mode (no int)
    loop {
        // Read output only if new values are available
        let all_sources = sensor.all_sources_get().unwrap();

        if all_sources.drdy_ah_qvar == 1 {
            let lsb = sensor.ah_qvar_raw_get().unwrap();
            let data = from_lsb_to_mv(lsb);

            writeln!(tx, "QVAR [mV]:{:.2}", data).unwrap();
        }
    }
}
