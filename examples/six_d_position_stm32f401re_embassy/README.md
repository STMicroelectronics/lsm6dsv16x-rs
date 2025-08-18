# LSM6DSV16X 6D Orientation Detection on STM32F401RE Nucleo-64 Using Embassy Framework

This example demonstrates how to detect 6D orientation events using the **LSM6DSV16X** accelerometer sensor on an **STM32F401RE** microcontroller board. The sensor is configured to generate interrupts on orientation changes, and the program outputs orientation events over UART.

The project uses the [Embassy](https://embassy.dev/) framework for peripheral initialization and interrupt handling, leveraging its async runtime to efficiently wait for sensor interrupts. UART output is performed using blocking writes.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer with 6D orientation detection
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for 6D orientation events

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from 6D event|

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The 6D event interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a delay abstraction.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt on rising edge for 6D event signaling.
- The interrupt pin is wrapped in Embassy's async `ExtiInput` to await rising edges asynchronously.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- Embedded functions are enabled.
- The 6D orientation detection is configured with a 60° threshold.
- Interrupt routing is configured to output 6D events on INT1 pin (PB4).
- Interrupt mode is set to level-triggered.
- Output data rate is set to 200 Hz with ±2g full scale and low-pass filter.

### Data Acquisition Loop

- The main async task waits for rising edge interrupts on PB4 signaling 6D orientation events.
- When an interrupt occurs, the program reads the sensor status.
- It decodes the 6D event bits and prints the detected orientation (e.g., "Y-axis UP", "X-axis DOWN") over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's 6D event interrupt output to PB4 on the STM32F401RE.
3. Build and flash the firmware onto the STM32F401RE board.
4. Open a serial terminal at 115200 baud on the USART2 TX line.
5. Change the orientation of the sensor to trigger 6D events.
6. Observe orientation event messages printed over UART.

---

## Notes

- The example uses Embassy's async runtime to efficiently wait for GPIO interrupts.
- UART output uses blocking writes without DMA.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `defmt` and `panic_probe`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [Embassy STM32 HAL](https://docs.rs/embassy-stm32)

---

*This README explains the embedded Rust program for 6D orientation detection on the LSM6DSV16X sensor using STM32F401RE and the Embassy framework.*
