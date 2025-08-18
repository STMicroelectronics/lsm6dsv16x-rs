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

use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;

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

    let mut msg: String<64> = String::new();

    let mut sensor = Lsm6dsv16x::new_i2c(i2c, I2CAddress::I2cAddH, delay.clone());

    // Check device ID
    let id = sensor.device_id_get().unwrap();
    if id != ID {
        writeln!(&mut msg, "Unexpected device ID: {}", id).unwrap();
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

            writeln!(&mut msg, "QVAR [mV]:{:.2}", data).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
