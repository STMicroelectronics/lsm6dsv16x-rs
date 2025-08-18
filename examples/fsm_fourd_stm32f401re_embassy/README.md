# LSM6DSV16X 4D Gesture Detection on STM32F401RE with Embassy Executor (UCF-Configured FSM Events)

This example demonstrates how to detect 4D gestures (X/Y up/down events) using the **LSM6DSV16X** inertial measurement unit (IMU) sensor interfaced with an **STM32F401RE** microcontroller board. The sensor's finite state machine (FSM) is configured via a UCF-generated register sequence to recognize these gestures.

The program uses the **Embassy** asynchronous executor framework for embedded Rust, leveraging async/await for interrupt-driven event handling. It communicates with the sensor over I2C and outputs detected gesture events over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope + FSM-based event detection)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for FSM event notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor FSM event |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's FSM event interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes the STM32F401RE peripherals using Embassy's HAL, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- The interrupt pin PB4 is configured as an input with external interrupt capability using Embassy's `ExtiInput`.
- A delay abstraction from Embassy is used for timing.

### Sensor Setup via UCF Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until the reset completes.
- The sensor is configured by applying a sequence of register writes and delays defined in the `FOUR_D` array, which is generated from a UCF file and programs the sensor's FSM for 4D gesture detection.
- This approach allows flexible and maintainable sensor configuration by editing UCF files and regenerating Rust code.

### Async Event Loop

- The main async task waits asynchronously for rising edges on the interrupt pin signaling FSM events.
- When an interrupt occurs, the program reads the sensor's status registers.
- If an FSM1 event is detected, the FSM output register is read.
- The FSM output is matched against known event codes:
  - `0x10` → "Y down event"
  - `0x20` → "Y up event"
  - `0x40` → "X down event"
  - `0x80` → "X up event"
- Detected events are printed over UART for monitoring.
- UART writes are performed in a blocking manner; DMA-based async writes are not used in this example.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's FSM interrupt line to PB4 on the STM32F401RE.
3. Build the Rust project; the UCF configuration is embedded in the code as the `FOUR_D` array.
4. Flash the compiled firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
6. Perform gestures that trigger FSM events.
7. Observe event notifications printed over UART.

---

## Notes

- This example uses Embassy's async executor to wait for GPIO interrupts asynchronously.
- UART output is done via blocking writes; DMA-based asynchronous UART transmission is not implemented here.
- The sensor FSM is configured via a UCF-generated register sequence embedded in the code.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to use `panic_probe` for debugging.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Embassy Embedded Rust Framework](https://embassy.dev/)
- [stm32h7xx-hal Rust crate (similar HAL concepts)](https://docs.rs/embassy-stm32)
- [lsm6dsv16x Rust driver crate](https://docs.rs/lsm6dsv16x)

---

*This README provides a detailed explanation of the embedded Rust program for FSM-based 4D gesture detection on STM32F401RE using Embassy executor and UCF-generated sensor configuration.*
