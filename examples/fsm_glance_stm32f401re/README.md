# LSM6DSV16X Glance and Deglance Gesture Detection on STM32F401RE Nucleo-64 (UCF-Configured FSM)

This example demonstrates how to detect *glance* and *deglance* gestures using the **LSM6DSV16X** accelerometer sensor on an **STM32F401RE** microcontroller board. The gestures are recognized by the sensor's finite state machine (FSM), configured via a UCF-generated register sequence. Detected events are output over UART.

The program uses the `lsm6dsv16x` Rust driver and applies a UCF configuration (`GLANCE_DETECTION`) to program the sensor's FSM. The example runs on the STM32F401RE with interrupts configured on PB4 to signal FSM events.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X Accelerometer with FSM support
- **Communication Interface:** I2C1 at 100 kHz Standard Mode (PB8 = SCL, PB9 = SDA)
- **UART:** USART2 for serial output at 115200 baud (PA2 TX)
- **Interrupt Pin:** PB4 configured as input with external interrupt for FSM event notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor FSM event |

The LSM6DSV16X sensor is connected via I2C1 on PB8/PB9. The FSM event interrupt line is connected to PB4, configured to trigger an external interrupt on rising edge. UART output is routed through PA2.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, UART, and a timer for delays.
- I2C1 is configured at 100 kHz Standard Mode on pins PB8 (SCL) and PB9 (SDA).
- UART is configured on USART2 (PA2) at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt on rising edge for FSM event detection.
- The external interrupt is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a mutex-protected static for safe interrupt flag clearing.

### Sensor Setup via UCF Configuration

- The LSM6DSV16X sensor is initialized over I2C with the high I2C address.
- The sensor is taken out of deep power down mode.
- The device ID is read and verified; if mismatched, the program halts.
- The sensor is reset to default configuration and waits until reset completes.
- The sensor is configured by applying a sequence of register writes and delays defined in the `GLANCE_DETECTION` array, generated from a UCF file. This programs the sensor's FSM for glance and deglance gesture detection.

### Data Acquisition Loop

- The program enters a low-power wait-for-interrupt (WFI) loop.
- When an FSM event interrupt occurs, the program reads the FSM status.
- If FSM1 event is detected, it reads the FSM output and matches known event codes:
  - `0x20` indicates a "GLANCE" event.
  - `0x08` indicates a "DEGLANCE" event.
- Detected events are printed over UART.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Connect the sensor's FSM interrupt output to PB4 on the STM32F401RE.
3. Build the project, which uses the **`ucf-tool`** to generate Rust configuration code from UCF files automatically at build time.
4. Flash the compiled Rust firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud on the UART port.
6. Perform glance and deglance gestures with the sensor.
7. Observe event notifications printed over UART.

---

## Notes

- This example uses polling of interrupts and FSM event registers for gesture detection.
- The **`ucf-tool`** enables flexible sensor FSM configuration by converting UCF files into Rust code.
- The sensor driver and UCF-generated code handle low-level register access and FSM programming.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for glance and deglance gesture detection on STM32F401RE using the LSM6DSV16X sensor and UCF-generated FSM configuration.*
