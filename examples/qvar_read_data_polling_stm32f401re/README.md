# LSM6DSV16X AH/QVAR Data Acquisition on STM32F401RE Nucleo (Polling Mode)

This example demonstrates how to acquire and output the **Angular Velocity Quadrature (AH/QVAR)** data from the **LSM6DSV16X** inertial measurement unit (IMU) sensor using an **STM32F401RE** microcontroller board. The sensor is configured to output accelerometer data at 15 Hz with specific filtering and block data update enabled.

The program communicates with the sensor over I2C, polls for new AH/QVAR data availability, reads the raw data, converts it to millivolts, and prints the results over UART.

The code is written in Rust using the `stm32f4xx-hal` hardware abstraction layer and the `lsm6dsv16x` sensor driver crate.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope + AH/QVAR feature)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- A delay abstraction is created using the system timer.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- The accelerometer output data rate is set to 15 Hz.
- The accelerometer full scale is set to ±2g.
- The filtering chain is configured:
  - Filter settling mask enables data ready signals for accelerometer and gyroscope.
  - Low-pass filter 2 is enabled with strong bandwidth.
- The Angular Velocity Quadrature (AH/QVAR) function is enabled.

### Data Acquisition Loop

- The program enters a polling loop.
- It checks if new AH/QVAR data is available by reading the sensor's status registers.
- When new data is ready, the raw AH/QVAR value is read.
- The raw value is converted from least significant bits (LSB) to millivolts (mV).
- The converted value is printed over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Build and flash the Rust firmware onto the STM32F401RE.
3. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
4. Observe the AH/QVAR data values printed continuously.

---

## Notes

- This example uses polling to read sensor data without interrupts.
- The AH/QVAR data is read only when new data is available, ensuring fresh samples.
- The sensor driver handles low-level register access and configuration.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).
- The function `from_lsb_to_mv` (not shown) should convert raw sensor LSB values to millivolts according to sensor datasheet scaling.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for AH/QVAR data acquisition on STM32F401RE using the LSM6DSV16X sensor.*
