# LSM6DSV16X Accelerometer Self-Test on STM32F401RE Nucleo-64

This example demonstrates how to perform a self-test on the **LSM6DSV16X** accelerometer sensor using an **STM32F401RE** microcontroller. The self-test verifies the sensor's functionality by comparing acceleration measurements with positive and negative self-test modes enabled.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer
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
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.

### Self-Test Procedure

- The self-test runs in a loop for both positive and negative self-test modes.
- For each mode:
  1. The sensor is put into soft power-down mode.
  2. FIFO is configured to store accelerometer-only data.
  3. The self-test sign bits are set according to the mode (positive or negative).
  4. Self-test is started with the appropriate configuration.
  5. The sensor is configured to output data at 200 Hz with ±8g full scale and low-pass filter.
  6. FIFO is enabled to collect samples.
  7. The first 5 samples are read from FIFO and averaged.
  8. The sensor is put back into power-down mode and FIFO is cleared.
  9. The self-test is switched to the other mode.
  10. The second set of 5 samples is read and averaged.
  11. The self-test is stopped.
  12. The deviation between the two averaged samples is computed.
  13. The deviation is checked against datasheet-defined ranges for each axis.
  14. The test result (PASS or FAIL) is printed over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe self-test results printed for positive and negative modes.

---

## Notes

- The self-test uses FIFO buffering to collect multiple samples for averaging.
- UART output uses blocking writes.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `panic_halt`.
- The self-test ranges are based on the LSM6DSV16X datasheet specifications.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README explains the embedded Rust program for performing accelerometer self-test on the LSM6DSV16X sensor using STM32F401RE.*
