# LSM6DSV16X Sensor Fusion FIFO Streaming on STM32F401RE Nucleo (Embassy Executor)

This example demonstrates how to configure and read sensor fusion data from the **LSM6DSV16X** inertial measurement unit (IMU) sensor using an **STM32F401RE** microcontroller board with the **Embassy** asynchronous executor framework. The sensor's dedicated **Sensor Fusion Low Power (SFLP)** block processes accelerometer and gyroscope data to generate:

- **Game rotation vector:** A quaternion representing the device's attitude.
- **Gravity vector:** A 3D vector representing the direction of gravity.
- **Gyroscope bias:** A 3D vector representing the gyroscope bias.

These data are streamed via the sensor's FIFO buffer and read by the microcontroller for further processing or transmission.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU with embedded sensor fusion (SFLP block)
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

- The program initializes STM32F401RE peripherals including clocks, GPIO pins, I2C, and UART using the Embassy HAL.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- A delay abstraction from Embassy is used for timing.

### Sensor Fusion (SFLP) Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- Accelerometer full scale is set to ±4g.
- Gyroscope full scale is set to ±2000 dps.
- FIFO watermark is set to 32 samples.
- FIFO is configured to batch sensor fusion data:
  - Game rotation vector (quaternion)
  - Gravity vector
  - Gyroscope bias
- FIFO mode is set to Stream Mode (continuous).
- Output data rates for accelerometer, gyroscope, and SFLP data are set to 30 Hz.
- Sensor fusion game rotation is enabled.
- Gyroscope bias offsets are initialized to zero (can be updated from stored calibration).

### FIFO Data Processing Loop

- The program continuously polls the FIFO status.
- When the FIFO watermark is reached, it reads all available FIFO samples.
- Each sample is decoded based on its tag:
  - `SflpGyroscopeBiasTag`: gyroscope bias in mdps (milli-degrees per second).
  - `SflpGravityVectorTag`: gravity vector in mg (milli-g).
  - `SflpGameRotationVectorTag`: game rotation quaternion, converted from half to single precision floats.
- Decoded data is printed over UART for monitoring.
- UART writes are performed in a blocking manner; DMA-based asynchronous UART transmission is not implemented in this example.

### Quaternion Conversion

- The helper function `sflp2q` converts the 3-component half-precision quaternion vector from the sensor into a normalized 4-component single-precision quaternion.
- It ensures normalization and handles cases where the sum of squares exceeds 1 due to rounding.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Build and flash the Rust firmware onto the STM32F401RE.
3. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
4. Observe FIFO sensor fusion data printed continuously, including gyroscope bias, gravity vector, and game rotation quaternion.

---

## Notes

- The SFLP block works at configurable output data rates up to 480 Hz; this example uses 30 Hz.
- The FIFO buffer stores tagged sensor data words (1 byte tag + 6 bytes data).
- The FIFO supports batching and compression to optimize data throughput.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_probe`).
- The example uses the `libm` crate for floating-point math functions.
- Conversion functions like `from_half_to_single_precision`, `from_fs125_to_mdps`, and `from_sflp_to_mg` are assumed to be provided by the sensor driver or user code.

---

## References

- STMicroelectronics, *AN5763: 6-axis IMU with embedded sensor fusion, AI, Qvar for high-end applications*, Rev 4.
- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Embassy Embedded Rust Framework](https://embassy.dev/)
- [Application Note - AN5763] (https://www.st.com/resource/en/application_note/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf)
---

*This README provides a detailed explanation of the embedded Rust program for sensor fusion data streaming on STM32F401RE using the LSM6DSV16X sensor and Embassy framework.*
