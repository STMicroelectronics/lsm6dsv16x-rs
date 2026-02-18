#[macro_export]
macro_rules! define_embassy_with_st_link {
    (
        i2c = {
            address: $sevenbit_address:expr,
            periph: $i2c_periph:ident,
            scl: $scl_pin:ident,
            sda: $sda_pin:ident,
            ev_irq: $i2c_ev_irq:ident,
            er_irq: $i2c_er_irq:ident,
            dma_tx: $i2c_dma_tx:ident,
            dma_rx: $i2c_dma_rx:ident,
        },
        uart = {
            periph: $usart_periph:ident,
            tx: $tx_pin:ident,
            dma_tx: $uart_dma_tx:ident,
            baud: $baud:literal,
        },
        $(int_pin = {
            pin: $int_pin:ident,
            exti_line: $exti_line:ident,
            exti_mux: $exti_mux:ident
        })?
    ) => {

        const I2CADDRESS: i2c::SevenBitAddress = $sevenbit_address;

        #[cfg(not(feature = "interrupt"))]
        #[allow(unused)]
        fn board_init(spawner: embassy_executor::Spawner)
            -> (
                impl embedded_hal_async::i2c::I2c,
                impl embedded_io::Write,
                embassy_time::Delay,
                (),
            )
        {
            let board = board::init_all(false);
            (board.i2c, board.uart, board.delay, ())
        }

        #[cfg(feature = "interrupt")]
        fn board_init(spawner: embassy_executor::Spawner)
            -> (
                impl embedded_hal_async::i2c::I2c,
                impl embedded_io::Write,
                embassy_time::Delay,
                impl crate::InterruptPin,
            )
        {
            let board = board::init_all(true);

            if let Some(pin) = board.int_pin {
                spawner.must_spawn(board::handle_interrupt_task(pin));
            }
            (board.i2c, board.uart, board.delay, board::InterruptPin{})
        }

        mod board {
            use embassy_stm32::{bind_interrupts, i2c, mode, peripherals, Config};
            use embassy_time::Delay;
            #[allow(unused)]
            use embassy_stm32::{gpio, exti::ExtiInput};
            use embassy_sync::signal::Signal;
            use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

            pub(crate) static INTERRUPT_SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();

            bind_interrupts!(pub struct Irqs {
                $i2c_ev_irq => i2c::EventInterruptHandler<peripherals::$i2c_periph>;
                $i2c_er_irq => i2c::ErrorInterruptHandler<peripherals::$i2c_periph>;
                $(
                    $exti_mux => embassy_stm32::exti::InterruptHandler<
                        embassy_stm32::interrupt::typelevel::$exti_mux
                    >;
                )?
            });

            #[allow(dead_code)]
            pub struct BoardResources {
                pub i2c: i2c::I2c<'static, mode::Async, i2c::mode::Master>,
                pub uart: embassy_stm32::usart::UartTx<'static, mode::Async>,
                pub delay: Delay,
                pub int_pin: Option<ExtiInput<'static>>,
            }

            #[allow(unused)]
            pub fn init_all(with_interrupt: bool) -> BoardResources {
                let p = embassy_stm32::init(Config::default());

                let mut config = i2c::Config::default();
                config.frequency = embassy_stm32::time::Hertz(100_000);
                //
                // I2C
                //
                let i2c = i2c::I2c::new(
                    p.$i2c_periph,
                    p.$scl_pin,
                    p.$sda_pin,
                    Irqs,
                    p.$i2c_dma_tx,
                    p.$i2c_dma_rx,
                    config,
                );

                //
                // UART TX
                //
                use embassy_stm32::usart;
                let mut usart_config: usart::Config = usart::Config::default();
                usart_config.baudrate = $baud;
                usart_config.data_bits = usart::DataBits::DataBits8;
                usart_config.parity = usart::Parity::ParityNone;

                let uart = usart::UartTx::new(
                    p.$usart_periph,
                    p.$tx_pin,
                    p.$uart_dma_tx,
                    usart_config,
                )
                .unwrap();

                //
                // Delay
                //
                let delay = Delay;

                //
                // Optional INT pin
                //
                let int_pin = {
                    #[allow(unused_mut)]
                    let mut maybe_pin = None;

                    $(
                    if with_interrupt {
                        use embassy_stm32::exti::ExtiInput;

                        let pin = ExtiInput::new(
                            p.$int_pin,
                            p.$exti_line,
                            gpio::Pull::None,
                            Irqs,
                        );
                        maybe_pin = Some(pin);
                    }
                    )?

                    maybe_pin
                };

                BoardResources {
                    i2c,
                    uart,
                    delay,
                    int_pin,
                }
            }

            #[allow(unused)]
            #[embassy_executor::task]
            pub async fn handle_interrupt_task(mut pin: embassy_stm32::exti::ExtiInput<'static>) {
                loop {
                    pin.wait_for_rising_edge().await;
                    INTERRUPT_SIGNAL.signal(());
                }
            }

            #[allow(unused)]
            pub struct InterruptPin;

            #[allow(unused)]
            impl crate::InterruptPin for InterruptPin {
                async fn wait_for_event(&mut self) {
                    INTERRUPT_SIGNAL.wait().await;
                }
            }
        }
    };
}

#[macro_export]
macro_rules! define_stm32_rs_with_st_link {
    (
        i2c = {
            address: $sevenbit_address:expr,
            periph: $i2c_periph:ident,
            scl: ($scl_port:ident, $scl_pin:ident),
            sda: ($sda_port:ident, $sda_pin:ident),
        },
        uart = {
            periph: $uart_periph:ident,
            tx: ($tx_port:ident, $tx_pin:ident),
        },
        $(interrupt = {
            pin: ($int_port:ident, $int_pin:ident),
            exti_irq: $exti_irq:ident
        })?
    ) => {
        const I2CADDRESS: u8 = $sevenbit_address;

        #[cfg(not(feature = "interrupt"))]
        fn board_init() -> (
            impl embedded_hal::i2c::I2c,
            impl embedded_io::Write,
            board::DelayHandle,
            ()
        ) {
            let board = board::init_all(false);
            (board.i2c, board.uart, board.delay, ())
        }

        #[cfg(feature = "interrupt")]
        fn board_init() -> (
            impl embedded_hal::i2c::I2c,
            impl embedded_io::Write,
            board::DelayHandle,
            impl crate::InterruptPin
        ) {
            let board = board::init_all(true);
            (board.i2c, board.uart, board.delay, board.int_pin)
        }

        mod board {
            #[allow(unused)]
            use stm32f4xx_hal::{
                gpio::{self, Edge, ExtiPin, gpioa, gpiob, gpioc},
                i2c::{I2c, Mode},
                pac::{self, interrupt, $i2c_periph, $uart_periph},
                prelude::*,
                serial::{config::Config, Serial},
                timer::Delay,
            };
            use core::cell::RefCell;
            use cortex_m::interrupt::Mutex;
            #[allow(unused)]
            use crate::{SerialWriter, InterruptPin};

            type IntPinType = gpio::ErasedPin<gpio::Input>;

            static DELAY: Mutex<RefCell<Option<Delay<pac::TIM1, 1000>>>> = Mutex::new(RefCell::new(None));
            #[allow(unused)]
            static INT_FLAG: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
            #[allow(unused)]
            static INT_PIN: Mutex<RefCell<Option<IntPinType>>> = Mutex::new(RefCell::new(None));

            // Internal Registry to hold split ports
            #[allow(unused)]
            struct PortRegistry {
                port_a: gpioa::Parts,
                port_b: gpiob::Parts,
                port_c: gpioc::Parts,
            }

            // Helper macro to map user tokens to the Registry fields
            macro_rules! get_port {
                (port_a, $reg:expr) => { $reg.port_a };
                (port_b, $reg:expr) => { $reg.port_b };
                (port_c, $reg:expr) => { $reg.port_c };
            }

            #[allow(dead_code)]
            pub struct BoardResources {
                pub i2c: I2c<$i2c_periph>,
                pub delay: DelayHandle,
                pub uart: SerialWriter<stm32f4xx_hal::serial::Tx<$uart_periph>>,
                pub int_pin: BlockingIntPin,
            }

            #[allow(unused)]
            pub fn init_all(is_interrupt: bool) -> BoardResources {
                let dp = pac::Peripherals::take().unwrap();
                let clocks = dp.RCC.constrain().cfgr.use_hse(8.MHz()).freeze();
                #[allow(unused)]
                let mut syscfg = dp.SYSCFG.constrain();

                // Split all ports exactly once
                let ports = PortRegistry {
                    port_a: dp.GPIOA.split(),
                    port_b: dp.GPIOB.split(),
                    port_c: dp.GPIOC.split(),
                };

                // I2C Setup - Accessing pins through the registry
                let scl = get_port!($scl_port, ports).$scl_pin.into_alternate_open_drain();
                let sda = get_port!($sda_port, ports).$sda_pin.into_alternate_open_drain();

                let i2c = I2c::new(
                    dp.$i2c_periph,
                    (scl, sda),
                    Mode::standard(400.kHz()),
                    &clocks,
                );

                let tim1_delay = dp.TIM1.delay_ms(&clocks);
                cortex_m::interrupt::free(|cs| {
                    *DELAY.borrow(cs).borrow_mut() = Some(tim1_delay);
                });

                // UART Setup
                let tx_pin = get_port!($tx_port, ports).$tx_pin.into_alternate();
                let tx = Serial::tx(
                    dp.$uart_periph,
                    tx_pin,
                    Config::default().baudrate(115_200.bps()),
                    &clocks,
                ).unwrap();

                // Interrupt Setup
                $(
                if is_interrupt {
                    let mut int_p = get_port!($int_port, ports).$int_pin.into_input().erase();
                    int_p.make_interrupt_source(&mut syscfg);
                    int_p.trigger_on_edge(&mut unsafe { pac::Peripherals::steal().EXTI }, Edge::Rising);
                    int_p.enable_interrupt(&mut unsafe { pac::Peripherals::steal().EXTI });

                    cortex_m::interrupt::free(|cs| {
                        INT_PIN.borrow(cs).replace(Some(int_p));
                    });

                    unsafe {
                        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::$exti_irq);
                    }
                }
                )?

                BoardResources {
                    i2c,
                    delay: DelayHandle::new(),
                    uart: SerialWriter(tx),
                    int_pin: BlockingIntPin,
                }
            }

            // ... (BlockingIntPin, $exti_irq interrupt, and DelayHandle implementations stay same)
            pub struct BlockingIntPin;
            $(
            impl InterruptPin for BlockingIntPin {
                fn wait_for_event(&mut self) {
                    while !cortex_m::interrupt::free(|cs| *INT_FLAG.borrow(cs).borrow()) { core::hint::spin_loop(); }
                    cortex_m::interrupt::free(|cs| { *INT_FLAG.borrow(cs).borrow_mut() = false; });
                }
            }
            #[interrupt]
            fn $exti_irq() {
                cortex_m::interrupt::free(|cs| {
                    if let Some(pin) = INT_PIN.borrow(cs).borrow_mut().as_mut() {
                        if pin.check_interrupt() {
                            pin.clear_interrupt_pending_bit();
                            *INT_FLAG.borrow(cs).borrow_mut() = true;
                        }
                    }
                });
            }
            )?

            #[derive(Clone, Copy)]
            pub struct DelayHandle;
            impl DelayHandle { pub fn new() -> Self { DelayHandle } }
            impl embedded_hal::delay::DelayNs for DelayHandle {
                fn delay_us(&mut self, us: u32) {
                    cortex_m::interrupt::free(|cs| {
                        if let Some(ref mut d) = *DELAY.borrow(cs).borrow_mut() { d.delay_us(us); }
                    });
                }
                fn delay_ns(&mut self, ns: u32) { self.delay_us((ns + 999) / 1_000); }
            }
        }
    };
}
