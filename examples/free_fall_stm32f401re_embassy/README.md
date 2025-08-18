# LSM6DSV16X Free-Fall Detection on STM32F401RE Nucleo-64

This example demonstrates how to detect free-fall events using the **LSM6DSV16X** accelerometer sensor on an **STM32F401RE** microcontroller board. The sensor is configured to generate an interrupt on free-fall detection, and the event is reported over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for free-fall event notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from free-fall event |

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The free-fall interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, and UART.
- The I2C bus is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- UART is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with no pull resistor and set up as an external interrupt line (EXTI4).
- An interrupt handler is bound to EXTI4 to handle free-fall interrupts.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- Block Data Update and interface increment are enabled for safe driver usage.
- Free-fall detection is configured with a duration of 10 and threshold of 312 mg.
- Interrupt routing is configured to output free-fall events on INT1 pin (PB4).
- Interrupt mode is set to latched.
- Output data rate is set to 200 Hz with 2x bandwidth low-pass filter and ±2g full scale.

### Data Acquisition Loop

- The program waits asynchronously for rising edge interrupts on PB4 signaling free-fall events.
- When a free-fall event is detected, a message "Free-Fall detected!" is printed over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's free-fall interrupt output to PB4 on the STM32F401RE.
3. Build and flash the firmware onto the STM32F401RE board.
4. Open a serial terminal at 115200 baud on the USART2 TX line.
5. Generate free-fall events by moving or dropping the sensor.
6. Observe free-fall detection messages printed over UART.

---

## Notes

- The example uses hardware interrupts to detect free-fall events efficiently.
- UART output uses blocking writes without DMA.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `defmt` and `panic_probe`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [embassy-stm32 HAL](https://docs.rs/embassy-stm32)

---

*This README provides a detailed explanation of the embedded Rust program for free-fall detection on STM32F401RE using the LSM6DSV16X sensor.*
