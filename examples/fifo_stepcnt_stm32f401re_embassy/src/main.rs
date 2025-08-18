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
    // Enable Block Data Update
    sensor.block_data_update_set(1).unwrap();
    let mut pin_int = PinIntRoute::default();
    pin_int.fifo_th = 1;
    sensor.pin_int1_route_set(&pin_int).unwrap();

    // // Enable interrupts
    // let irq: InterruptMode = InterruptMode {
    //     enable: 1,
    //     lir: 1,
    // };
    //sensor.interrupt_enable_set(irq).unwrap();
    // Set full scale
    sensor.xl_full_scale_set(XlFullScale::_2g).unwrap();
    // Set FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to FIFO_WATERMARK samples
    sensor.fifo_watermark_set(25).unwrap();

    // Enable pedometer
    let stpcnt_mode: StpcntMode = StpcntMode {
        step_counter_enable: 1,
        false_step_rej: 1,
    };
    sensor.stpcnt_mode_set(stpcnt_mode).unwrap();
    // Enable step counter data in FIFO
    sensor.fifo_stpcnt_batch_set(1).unwrap();

    // Set FIFO batch XL/Gyro ODR to 12.5hz
    sensor.fifo_xl_batch_set(FifoBatch::_60hz).unwrap();

    // Set FIFO mode to Stream mode
    sensor.fifo_mode_set(FifoMode::StreamMode).unwrap();

    // Set Output Data Rate
    sensor.xl_data_rate_set(Odr::_60hz).unwrap();

    sensor
        .fifo_timestamp_batch_set(FifoTimestampBatch::Dec32)
        .unwrap();
    sensor.timestamp_set(1).unwrap();

    loop {
        interrupt.wait_for_rising_edge().await;
        let status: AllSources = sensor.all_sources_get().unwrap();
        let fifo_status: FifoStatus = sensor.fifo_status_get().unwrap();
        let num = fifo_status.fifo_level;
        if status.fifo_th == 1 {
            writeln!(&mut msg, "-- FIFO num {}\r\n", num).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();

            for _ in 0..num {
                let f_data: FifoOutRaw = sensor.fifo_out_raw_get().unwrap();
                let steps = i16::from_le_bytes([f_data.data[0], f_data.data[1]]);
                let ts = u32::from_le_bytes([
                    f_data.data[2],
                    f_data.data[3],
                    f_data.data[4],
                    f_data.data[5],
                ]);
                match f_data.tag {
                    Tag::StepCounterTag => {
                        writeln!(
                            &mut msg,
                            "Steps:\t{:.2}\t{:.2}us\t\r\n",
                            steps,
                            from_lsb_to_nsec(ts) / 1000.0,
                        )
                        .unwrap();
                        let _ = tx.blocking_write(msg.as_bytes());
                        msg.clear();
                    }
                    _ => {}
                }
            }

            writeln!(&mut msg, "------ \r\n\r\n").unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }
}
