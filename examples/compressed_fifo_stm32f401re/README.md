# LSM6DSV16X FIFO Data Acquisition on STM32F401RE Nucleo-64

This example demonstrates how to acquire and decode FIFO data from the **LSM6DSV16X** inertial measurement unit (IMU) sensor on an **STM32F401RE** microcontroller board. The sensor streams accelerometer and gyroscope data into its FIFO buffer with compression enabled, and the program reads, decompresses, and outputs the sensor data over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                    |
|--------------|-----------------|-------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)   |
| I2C1_SDA     | PB9             | I2C data line (open-drain)    |
| USART2_TX    | PA2             | UART transmit for debug output|

The LSM6DSV16X sensor is connected to the STM32F401RE via I2C1 on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a delay abstraction.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.

### FIFO Configuration and Data Acquisition

- The sensor's FIFO is configured with:
  - Watermark set to 32 samples.
  - FIFO batch rate for accelerometer and gyroscope set to 15 Hz.
  - FIFO mode set to stream (continuous) mode.
  - FIFO compression enabled (8:1 compression).
  - Timestamp batching and timestamp generation enabled.
- Output data rate for accelerometer and gyroscope is set to 15 Hz.
- The program continuously polls the FIFO watermark flag.
- When the watermark is reached, it reads all FIFO entries.
- FIFO data is decoded and sorted using the `fifo_tool` crate.
- Accelerometer and gyroscope samples are extracted and converted to physical units (mg and mdps).
- Sensor data with timestamps and tags are printed over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe decoded accelerometer and gyroscope data with timestamps printed over UART.

---

## Notes

- The example uses polling to check FIFO watermark status.
- FIFO compression reduces data size for efficient storage.
- UART output uses blocking writes.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `panic_halt`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README explains the embedded Rust program for FIFO-based sensor data acquisition on the LSM6DSV16X using STM32F401RE.*
