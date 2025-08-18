#![no_std]
#![no_main]

use panic_halt as _;

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use lsm6dsv16x_rs::prelude::*;
use lsm6dsv16x_rs::*;
use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::{config::Config, Serial},
};
type IntPin = gpio::PB4<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
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
    let mut int_pin = gpiob.pb4.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

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
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }
        let status: AllSources = sensor.all_sources_get().unwrap();
        let fifo_status: FifoStatus = sensor.fifo_status_get().unwrap();
        let num = fifo_status.fifo_level;
        if status.fifo_th == 1 {
            writeln!(tx, "-- FIFO num {}\r\n", num).unwrap();

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

#[interrupt]
fn EXTI4() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    });
}
