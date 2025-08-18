#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
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

    // Configure the interrupt pin (if needed) and obtain handler.
    // On the Nucleo FR401 the interrupt pin is connected to pin PB0.
    let interrupt = Input::new(p.PB4, Pull::None);
    let mut interrupt = ExtiInput::new(interrupt, p.EXTI4);

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

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

    let mut pin_int = PinIntRoute::default();
    pin_int.wakeup = 1;
    sensor.pin_int1_route_set(&pin_int).unwrap();

    let irq = InterruptMode { enable: 1, lir: 1 };
    sensor.interrupt_enable_set(irq).unwrap();

    sensor.filt_xl_fast_settling_set(1).unwrap();
    sensor.mask_trigger_xl_settl_set(1).unwrap();
    sensor
        .filt_wkup_act_feed_set(FiltWkupActFeed::HighPass)
        .unwrap();

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

    sensor.act_thresholds_set(threshold).unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_480hz).unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();

    loop {
        // Wait for interrupt
        interrupt.wait_for_high().await;
        let status = sensor.all_sources_get().unwrap();
        if status.wake_up == 1 {
            writeln!(&mut msg, "Wakeup event").unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
