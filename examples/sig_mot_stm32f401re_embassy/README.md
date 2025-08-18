# LSM6DSV16X Significant Motion Detection on STM32F401RE Nucleo

This example demonstrates how to use the **significant motion** feature of the **LSM6DSV16X** inertial measurement unit (IMU) sensor on an **STM32F401RE** microcontroller board. The significant motion function detects meaningful user movement, such as a change in location, using only the accelerometer. It generates an interrupt when a threshold of steps is exceeded, enabling power-efficient location-based applications.

The program configures the sensor to enable significant motion detection at 120 Hz accelerometer output data rate (ODR), routes the significant motion interrupt to an external pin, and outputs event notifications over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer with embedded significant motion detection)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for significant motion event

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from significant motion event |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's significant motion interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes STM32F401RE peripherals including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt capability on rising edge.
- The external interrupt line is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a global mutex-protected static to safely clear interrupt flags.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency during reads.
- The significant motion detection function is enabled by setting the `SIGN_MOTION_EN` bit.
- The significant motion interrupt is routed to INT1 pin (PB4) by configuring embedded function interrupt routing.
- Embedded interrupt latch mode is enabled to keep the interrupt signal asserted until cleared by software.
- The accelerometer output data rate is set to 120 Hz.
- The accelerometer full scale is set to ±2g.

### Event Loop

- The program asynchronously waits for rising edge interrupts on the significant motion interrupt pin.
- When a significant motion event occurs, the sensor sets the corresponding embedded status bit.
- The program reads the embedded status register and prints "Sig motion event" over UART when detected.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's significant motion interrupt line to PB4 on the STM32F401RE.
3. Build and flash the Rust firmware onto the STM32F401RE.
4. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
5. Move the device to trigger significant motion detection.
6. Observe "Sig motion event" messages printed over UART when significant motion is detected.

---

## Notes

- The significant motion function works at 30 Hz or higher accelerometer ODR; this example uses 120 Hz.
- The function generates an interrupt when the step count difference exceeds 10 steps, then resets internally.
- Interrupt signal behavior is pulsed by default; latch mode is enabled here to keep the interrupt asserted until cleared.
- The embedded step counter algorithm is automatically enabled with significant motion detection.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to use `panic_probe` for debugging.

---

## References

- STMicroelectronics, *DT0155: Synchronizing multiple sensors using ODR-triggered mode in MEMS devices*, February 2024.
- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Application Note - AN5763] (https://www.st.com/resource/en/application_note/an5763-lsm6dsv16x-6axis-imu-with-embedded-sensor-fusion-ai-qvar-for-highend-applications-stmicroelectronics.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for significant motion detection on STM32F401RE using the LSM6DSV16X sensor.*
