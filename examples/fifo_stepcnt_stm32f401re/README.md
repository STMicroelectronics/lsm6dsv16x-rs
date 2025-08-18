# LSM6DSV16X Step Counter with FIFO on STM32F401RE Nucleo-64

This example demonstrates how to use the **LSM6DSV16X** accelerometer sensor's embedded step counter feature with FIFO buffering on an **STM32F401RE** microcontroller board. The program configures the sensor to count steps, buffers step data in FIFO, and outputs step counts and timestamps over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer with embedded step counter and FIFO
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for FIFO watermark signaling

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from FIFO watermark |

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The FIFO watermark interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a timer for delays.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt on rising edge for FIFO watermark signaling.
- The external interrupt is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a mutex-protected static for safe interrupt flag clearing.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- Embedded functions are enabled, including the step counter.
- Step counter debounce and mode are configured to reject false steps and enable step counting with FIFO storage.
- FIFO is configured in stream mode with double FIFO storage and watermark set to 8 entries.
- Timestamping is enabled.
- Interrupt routing is configured to output FIFO watermark interrupts on INT1 pin (PB4).
- Output data rate is set to 25 Hz with ±4g full scale and low-pass filter.

### Data Acquisition Loop

- The program enters a low-power wait-for-interrupt (WFI) loop.
- When a FIFO watermark interrupt occurs, the program reads the number of FIFO entries.
- It reads FIFO data entries and processes step counter tags.
- Step counts and timestamps (in milliseconds) are printed over UART.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's FIFO watermark interrupt output to PB4 on the STM32F401RE.
3. Build and flash the firmware onto the STM32F401RE board.
4. Open a serial terminal at 115200 baud on the USART2 TX line.
5. Move or walk with the sensor to generate step events.
6. Observe step counts and timestamps printed over UART.

---

## Notes

- The example uses hardware interrupts and FIFO buffering to efficiently handle step counting.
- UART output uses blocking writes without DMA.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `panic_halt`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README explains the embedded Rust program for step counting with FIFO on the LSM6DSV16X sensor using STM32F401RE.*
