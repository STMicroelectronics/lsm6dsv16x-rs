#![no_std]
#![doc = include_str!("../README.md")]
#![allow(clippy::duplicate_mod)]

#[cfg(feature = "async")]
#[path = "."]
pub mod asynchronous {
    use bisync::asynchronous::*;
    use embedded_hal_async::delay::DelayNs;
    use embedded_hal_async::i2c::{I2c, SevenBitAddress};
    use embedded_hal_async::spi::SpiDevice;
    use st_mems_bus::asynchronous::*;

    pub mod driver;
    pub mod prelude;
    pub mod register;

    pub use driver::*;
}

#[cfg(feature = "blocking")]
#[path = "."]
pub mod blocking {
    use bisync::synchronous::*;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{I2c, SevenBitAddress};
    use embedded_hal::spi::SpiDevice;
    use st_mems_bus::blocking::*;

    pub mod driver;
    pub mod prelude;
    pub mod register;

    pub use driver::*;
}
