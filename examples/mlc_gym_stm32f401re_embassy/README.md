# LSM6DSV16X 4D Gesture Detection on STM32F401RE Nucleo with I2C and External Interrupt (Embassy Executor)

This example demonstrates how to detect 4D gestures (X/Y up/down events) using the **LSM6DSV16X** inertial measurement unit (IMU) sensor interfaced with an **STM32F401RE** microcontroller board. The sensor is configured via a UCF-generated register sequence to recognize these gestures.

The program uses the **Embassy** asynchronous executor framework for embedded Rust, communicating with the sensor over I2C and outputting detected gesture events over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for event notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor event |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's event interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes STM32F401RE peripherals using Embassy's HAL, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- The interrupt pin PB4 is configured as an input with external interrupt capability.
- A delay abstraction from Embassy is used for timing.

### Sensor Setup via UCF Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- The sensor is configured by applying a sequence of register writes and delays defined in the `FOUR_D` array, which is generated from a UCF file and programs the sensor for 4D gesture detection.
- This approach allows flexible and maintainable sensor configuration by editing UCF files and regenerating Rust code.

### Event Loop

- The main async task waits for rising edges on the interrupt pin signaling sensor events.
- When an interrupt occurs, the program reads the sensor's status registers.
- If an event is detected, the output register is read.
- The output is matched against known event codes:
  - `0x10` → "Y down event"
  - `0x20` → "Y up event"
  - `0x40` → "X down event"
  - `0x80` → "X up event"
- Detected events are printed over UART for monitoring.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's interrupt line to PB4 on the STM32F401RE.
3. Build the Rust project; the UCF configuration is embedded in the code as the `FOUR_D` array.
4. Flash the compiled firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
6. Perform gestures that trigger sensor events.
7. Observe event notifications printed over UART.

---

## Notes

- This example uses Embassy's async executor to wait for GPIO interrupts.
- UART output is done via blocking writes; DMA-based asynchronous UART transmission is not implemented here.
- The sensor is configured via a UCF-generated register sequence embedded in the code.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to use `panic_probe` for debugging.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Embassy Embedded Rust Framework](https://embassy.dev/)

---

*This README provides a detailed explanation of the embedded Rust program for 4D gesture detection on STM32F401RE using the LSM6DSV16X sensor and UCF-generated configuration code.*
