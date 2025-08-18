# LSM6DSV16X Accelerometer and Temperature Data Acquisition on STM32F401RE Nucleo-64 Using Embassy Framework

This example demonstrates how to read acceleration and temperature data from the **LSM6DSV16X** sensor using an **STM32F401RE** microcontroller board. The sensor is configured to operate in polling mode without interrupts, and the program outputs sensor readings over UART.

The project uses the [Embassy](https://embassy.dev/) framework for peripheral initialization and interrupt handling. The async runtime efficiently waits for GPIO interrupts signaling new sensor data. UART output is performed using blocking writes.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for data-ready signaling

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor data-ready signal |

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The data-ready interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a delay abstraction.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt on rising edge for data-ready signaling.
- The interrupt pin is wrapped in Embassy's async `ExtiInput` to await rising edges asynchronously.

### Sensor Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- Block Data Update and interface increment are enabled for safe driver usage.
- Interrupt routing is configured to output data-ready signals on INT1 pin (PB4).
- Output data rate is set to 25 Hz with ±4g full scale and low-pass filter.

### Data Acquisition Loop

- The main async task waits for rising edge interrupts on PB4 signaling new sensor data.
- When an interrupt occurs, the program reads acceleration and temperature data.
- Sensor readings are printed over UART.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's data-ready interrupt output to PB4 on the STM32F401RE.
3. Build and flash the firmware onto the STM32F401RE board.
4. Open a serial terminal at 115200 baud on the USART2 TX line.
5. Observe acceleration and temperature readings printed over UART.

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

*This README explains the embedded Rust program for accelerometer and temperature data acquisition on the LSM6DSV16X sensor using STM32F401RE and the Embassy framework.*
