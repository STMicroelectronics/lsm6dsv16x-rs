# LSM6DSV16X 4D Gesture Detection on STM32F401RE Nucleo with I2C and External Interrupt

This example demonstrates how to interface the **LSM6DSV16X** inertial measurement unit (IMU) sensor with an **STM32F401RE** microcontroller board to detect 4D gestures (X/Y up/down events) using the sensor's finite state machine (FSM) capabilities.

The program configures the sensor via a predefined UCF-generated register sequence, sets up an external interrupt on a GPIO pin connected to the sensor's interrupt line, and outputs detected gesture events over UART.

The code is written in Rust using the `stm32f4xx-hal` hardware abstraction layer and the `lsm6dsv16x` sensor driver crate. It leverages a UCF configuration embedded in the code to program the sensor FSM for gesture detection.

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

- The microcontroller peripherals are initialized, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with an external interrupt on the rising edge.
- The external interrupt is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a global mutex-protected static for safe access in the interrupt handler.

### Sensor Setup via UCF Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until the reset completes.
- The sensor is configured by applying a sequence of register writes and delays defined in the `FOUR_D` array, which is generated from a UCF file and programs the sensor's FSM for 4D gesture detection.
- This approach allows flexible and maintainable sensor configuration by editing UCF files and regenerating Rust code.

### Main Loop and Event Handling

- The program enters a low-power wait-for-interrupt (`wfi`) loop.
- When the sensor triggers an interrupt (FSM event), the program reads the sensor's status registers.
- If an FSM1 event is detected, the FSM output register is read.
- The FSM output is matched against known event codes:
  - `0x10` → "Y down event"
  - `0x20` → "Y up event"
  - `0x40` → "X down event"
  - `0x80` → "X up event"
- Detected events are printed over UART for monitoring.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

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

- The example uses a blocking wait-for-interrupt loop to detect FSM events.
- The sensor FSM is configured via a UCF-generated register sequence embedded in the code.
- The interrupt handler only clears the interrupt flag; all sensor communication occurs in the main loop.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for 4D gesture detection on STM32F401RE using the LSM6DSV16X sensor and UCF-generated FSM configuration.*
