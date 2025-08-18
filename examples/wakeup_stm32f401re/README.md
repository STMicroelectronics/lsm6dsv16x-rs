# LSM6DSV16X Wakeup Event Detection on STM32F401RE Nucleo-64

This example demonstrates how to detect wakeup events using the **LSM6DSV16X** accelerometer sensor on an **STM32F401RE** microcontroller board. The sensor is configured to generate interrupts on wakeup events, and the program outputs wakeup notifications over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer with wakeup detection capability
- **Communication Interface:** I2C1 at 100 kHz Standa Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for wakeup events

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from wakeup event |

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The wakeup event interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a timer for delays.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt on rising edge for wakeup event signaling.
- The external interrupt is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a mutex-protected static for safe interrupt flag clearing.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- Block Data Update and interface increment are enabled for safe driver usage.
- Wakeup configuration is set with:
  - Wake duration: 0 ODR cycles
  - Sleep duration: 0
  - Wake threshold weight: 1
  - Wake threshold: 2
  - Wake enable: Sleep on
  - Inactivity ODR: No change
- Interrupt routing is configured to output wakeup events on INT1 pin (PB4).
- Interrupt mode is set to latched.
- Output data rate is set to 200 Hz with ±2g full scale and low-pass filter.

### Data Acquisition Loop

- The program enters a low-power wait-for-interrupt (WFI) loop.
- When a wakeup event interrupt occurs, the program reads the sensor status.
- If a wakeup event is detected, a message "WAKEUP event detected" is printed over UART.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's wakeup event interrupt output to PB4 on the STM32F401RE.
3. Build and flash the firmware onto the STM32F401RE board.
4. Open a serial terminal at 115200 baud on the USART2 TX line.
5. Generate wakeup events by moving or shaking the sensor.
6. Observe wakeup event messages printed over UART.

---

## Notes

- The example uses hardware interrupts to efficiently detect wakeup events.
- UART output uses blocking writes without DMA.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `panic_halt`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README explains the embedded Rust program for wakeup event detection on the LSM6DSV16X sensor using STM32F401RE.*
