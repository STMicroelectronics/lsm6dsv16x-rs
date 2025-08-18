# LSM6DSV16X Tap Detection on STM32F401RE Nucleo (Embassy Executor)

This example demonstrates how to configure and detect **single** and **double tap** events using the **LSM6DSV16X** inertial measurement unit (IMU) sensor on an **STM32F401RE** microcontroller board. The sensor's embedded tap detection algorithm is used to generate interrupts on tap events, which are then reported over UART.

The program configures the sensor to detect taps on the Z-axis, sets appropriate thresholds and timing windows, and outputs tap event notifications over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer with tap detection)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for tap events

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from tap event|

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's tap event interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes STM32F401RE peripherals including clocks, GPIO pins, I2C, and UART using the Embassy HAL.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with external interrupt capability.
- The external interrupt line is enabled in the NVIC and linked to the EXTI4 interrupt handler.

### Tap Detection Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- Single and double tap interrupts are enabled and routed to INT1 pin (PB4).
- Tap detection is enabled only on the Z-axis.
- Tap thresholds and timing windows (shock, quiet, tap gap) are configured to tune detection sensitivity and timing.
- Tap mode is set to detect both single and double taps.
- Accelerometer output data rate is set to 480 Hz.
- Accelerometer full scale is set to ±8g.

### Event Loop

- The main async task waits asynchronously for rising edge interrupts on the tap interrupt pin.
- When a tap event occurs, the sensor sets the corresponding status bits.
- The program reads the sensor's status and prints "Single TAP" or "Double TAP" over UART accordingly.
- UART writes are performed in a blocking manner; DMA-based asynchronous UART transmission is not implemented.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's tap interrupt line to PB4 on the STM32F401RE.
3. Build and flash the Rust firmware onto the STM32F401RE.
4. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
5. Perform single or double tap gestures on the sensor.
6. Observe "Single TAP" or "Double TAP" messages printed over UART when taps are detected.

---

## Notes

- Tap detection is enabled only on the Z-axis in this example; other axes can be enabled by modifying the configuration.
- Tap thresholds and timing windows can be adjusted to tune sensitivity and debounce behavior.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to use `panic_probe` for debugging.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Embassy Embedded Rust Framework](https://embassy.dev/)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for tap detection on STM32F401RE using the LSM6DSV16X sensor and Embassy framework.*
