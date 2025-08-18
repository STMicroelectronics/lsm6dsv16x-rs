# LSM6DSV16X Accelerometer Data Acquisition on STM32F401RE Nucleo with External Interrupt

This example demonstrates how to acquire accelerometer data from the **LSM6DSV16X** inertial measurement unit (IMU) sensor using an **STM32F401RE** microcontroller board. The sensor is configured to output accelerometer data at 120 Hz with filtering enabled, and data-ready signals are routed to an external interrupt pin.

The program communicates with the sensor over I2C, configures the sensor and interrupt pin, and outputs acceleration data over UART whenever new data is available.

The code is written in Rust using the `stm32f4xx-hal` hardware abstraction layer and the `lsm6dsv16x` sensor driver crate.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for accelerometer data-ready signal

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor data-ready signal |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's accelerometer data-ready interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt capability on rising edge.
- The external interrupt line is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a global mutex-protected static to safely clear interrupt flags in the handler.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- The accelerometer data-ready signal is routed to interrupt pin INT1 (PB4).
- The accelerometer output data rate is set to 120 Hz.
- The accelerometer full scale is set to ±2g.
- The filtering chain is configured:
  - Filter settling mask enables data ready signals for accelerometer, gyroscope, and OIS.
  - Gyroscope low-pass filter 1 is enabled with ultra-light bandwidth.
  - Accelerometer low-pass filter 2 is enabled with strong bandwidth.

### Data Acquisition Loop

- The program enters a low-power wait-for-interrupt (`wfi`) loop.
- When the accelerometer data-ready interrupt occurs, the program reads the sensor's data-ready flags.
- If new accelerometer data is available, raw acceleration data is read.
- Raw data is converted from sensor units to milli-g (mg).
- Converted acceleration values for X, Y, and Z axes are printed over UART.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's accelerometer data-ready interrupt line to PB4 on the STM32F401RE.
3. Build and flash the Rust firmware onto the STM32F401RE.
4. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
5. Observe acceleration data printed continuously whenever new data is available.

---

## Notes

- This example uses an external interrupt to detect new accelerometer data availability.
- The sensor driver handles low-level register access and configuration.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).
- Conversion function `from_fs2_to_mg` (not shown) converts raw sensor data to mg units.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for accelerometer data acquisition on STM32F401RE using the LSM6DSV16X sensor with external interrupt support.*
