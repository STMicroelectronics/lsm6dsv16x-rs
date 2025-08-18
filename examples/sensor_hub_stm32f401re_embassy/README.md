# LSM6DSV16X Sensor Hub with LIS2MDL Magnetometer and LPS22DF Barometer on STM32F401RE Nucleo (Embassy Executor)

This example demonstrates how to use the **LSM6DSV16X** inertial measurement unit (IMU) as a sensor hub to interface with external sensors—**LIS2MDL** magnetometer and **LPS22DF** barometer—on an **STM32F401RE** microcontroller board. The LSM6DSV16X reads data from these external sensors via its embedded sensor hub feature and streams combined data through its FIFO buffer.

The program configures the LSM6DSV16X and external sensors, sets up FIFO streaming with watermark interrupts, and outputs sensor data including accelerometer, magnetometer, barometer, and timestamps over UART.

The code is written in Rust using the Embassy embedded framework, the `embassy-stm32` hardware abstraction layer, and the `lsm6dsv16x`, `lis2mdl`, and `lps22df` sensor driver crates.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensors:**
  - LSM6DSV16X 6-axis IMU with embedded sensor hub
  - LIS2MDL 3-axis magnetometer (connected via LSM6DSV16X sensor hub)
  - LPS22DF barometer (connected via LSM6DSV16X sensor hub)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for FIFO watermark

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from FIFO watermark |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The LIS2MDL and LPS22DF sensors are connected to the LSM6DSV16X sensor hub internally. The FIFO watermark interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes STM32F401RE peripherals including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt capability on rising edge.
- The external interrupt line is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a global mutex-protected static to safely clear interrupt flags.

### Sensor Hub and External Sensors Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The LIS2MDL magnetometer and LPS22DF barometer are initialized via the LSM6DSV16X sensor hub interface.
- Device IDs of all sensors are verified.
- Sensors are reset to default configurations and configured for continuous measurement modes:
  - LIS2MDL set to continuous mode at 20 Hz.
  - LPS22DF configured with 25 Hz ODR, averaging, and low-pass filtering.
- The LSM6DSV16X FIFO watermark is set to 64 samples.
- FIFO batching is configured to include accelerometer and gyroscope data at 60 Hz.
- FIFO mode is set to Stream Mode (continuous).
- FIFO watermark interrupt is routed to INT1 pin.
- Timestamping is enabled with 32-sample decimation.
- Accelerometer data rate is initially disabled, then set to 120 Hz to trigger sensor hub operations.
- Filtering chain is configured for accelerometer data.

### Sensor Hub Mode (Mode 2) Overview

- The LSM6DSV16X operates in sensor hub mode (mode 2), allowing connection of up to 4 external sensors via its I2C master interface.
- Sensor hub trigger can be synchronized with accelerometer/gyroscope data-ready signals or an external signal on INT2 pin.
- Sensor hub registers configure slave addresses, register addresses, read/write operations, and number of bytes to read/write.
- External sensor data are stored in SENSOR_HUB_x registers and can be batched into FIFO.
- Pass-through mode enables direct access to external sensor registers for configuration if needed.

### Data Acquisition Loop

- The program asynchronously waits for rising edge interrupts on the FIFO watermark pin.
- When FIFO watermark is reached, it reads all FIFO samples.
- Each FIFO sample is decoded based on its tag:
  - `XlNcTag`: accelerometer data converted to mg.
  - `TimestampTag`: timestamp converted to microseconds.
  - `SensorhubSlave0Tag`: LIS2MDL magnetometer data converted to milliGauss.
  - `SensorhubSlave1Tag`: LPS22DF barometer pressure (hPa) and temperature (°C).
- Decoded sensor data is printed over UART.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor and expansion sensors (LIS2MDL, LPS22DF) to the STM32F401RE Nucleo board.
2. Connect the FIFO watermark interrupt line to PB4 on the STM32F401RE.
3. Build and flash the Rust firmware onto the STM32F401RE.
4. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
5. Observe combined sensor data streamed via FIFO and printed over UART.

---

## Notes

- This example uses the LSM6DSV16X sensor hub feature to read external sensors transparently.
- FIFO watermark interrupts enable efficient batch reading of sensor and sensor hub data.
- Sensor hub mode (mode 2) allows synchronized and sequential I2C transactions with multiple external sensors.
- Pass-through mode can be used for direct external sensor configuration if needed.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).
- UART writes are blocking; asynchronous DMA-based UART transmission is not implemented.

---

## References

- STMicroelectronics, *AN5763: 6-axis IMU with embedded sensor fusion, AI, Qvar for high-end applications*, Rev 4.
- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
- [LPS22DF Datasheet](https://www.st.com/resource/en/datasheet/lps22df.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)
- [Application Note - AN5763] (https://www.st.com/resource/en/application_note/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf)

---

*This README provides a detailed explanation of the embedded Rust program for sensor hub data acquisition on STM32F401RE using the LSM6DSV16X sensor with LIS2MDL and LPS22DF external sensors, leveraging sensor hub mode (mode 2) as described in STMicroelectronics AN5763.*
