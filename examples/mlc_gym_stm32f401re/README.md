# LSM6DSV16X Gym Activity Recognition on STM32F401RE Nucleo (UCF-Configured MLC Events)

This example demonstrates how to detect gym activities such as *biceps curl*, *lateral raises*, and *squats* using the **LSM6DSV16X** inertial measurement unit (IMU) sensor with its Machine Learning Core (MLC) feature. The sensor is configured via a UCF-generated register sequence to recognize these activities.

The project interfaces the **LSM6DSV16X** sensor with an **STM32F401RE** microcontroller board over I2C. It configures the sensor using a UCF file converted into Rust code, handles interrupts triggered by the sensor's MLC events, and outputs detected activity events over UART.

The code is written in Rust using the `stm32f4xx-hal` hardware abstraction layer and the `lsm6dsv16x` sensor driver crate. The UCF configuration is embedded as the `GYM` array, enabling flexible sensor programming.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LSM6DSV16X IMU (accelerometer + gyroscope + Machine Learning Core)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB4 configured as input with external interrupt for MLC event notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                      |
|--------------|-----------------|---------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)     |
| I2C1_SDA     | PB9             | I2C data line (open-drain)      |
| USART2_TX    | PA2             | UART transmit for debug output  |
| EXTI4 (INT)  | PB4             | External interrupt from sensor MLC event |

The LSM6DSV16X sensor is connected to the STM32F401RE via the I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's MLC event interrupt line is connected to PB4, configured to trigger an external interrupt on the rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals, including clocks, GPIO pins, I2C, and UART.
- I2C1 is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- USART2 is configured on PA2 at 115200 baud for serial output.
- PB4 is configured as an input pin with interrupt capability on rising edge.
- The external interrupt line is enabled in the NVIC and linked to the EXTI4 interrupt handler.
- The interrupt pin is stored in a global mutex-protected static to safely clear interrupt flags in the handler.

### Sensor Setup via UCF Configuration

- The LSM6DSV16X sensor is initialized over I2C.
- The device ID is read and verified to confirm sensor presence.
- The sensor is reset to default configuration and the program waits until reset completes.
- The sensor is configured by applying a sequence of register writes and delays defined in the `GYM` array, generated from a UCF file, which programs the sensor's MLC for gym activity recognition.
- This approach allows flexible and maintainable sensor configuration by editing UCF files and regenerating Rust code.

### Main Loop and Event Handling

- The program enters a low-power wait-for-interrupt (`wfi`) loop.
- When the sensor triggers an interrupt (MLC event), the program reads the sensor's status registers.
- If an MLC1 event is detected, the MLC output register is read.
- The MLC output is matched against known event codes:
  - `4` → "biceps curl event"
  - `8` → "Lateral raises event"
  - `12` → "Squats event"
- Detected events are printed over UART for monitoring.

### Interrupt Handler

- The `EXTI4` interrupt handler clears the interrupt pending bit on PB4 to allow further interrupts.

---

## Usage

1. Connect the LSM6DSV16X sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's MLC interrupt line to PB4 on the STM32F401RE.
3. Build the Rust project; the UCF configuration is embedded in the code as the `GYM` array.
4. Flash the compiled firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud on the USART2 TX pin (PA2).
6. Perform gym activities that trigger MLC events.
7. Observe event notifications printed over UART.

---

## Notes

- This example uses polling of interrupts and MLC event registers for activity detection.
- The sensor is configured via a UCF-generated register sequence embedded in the code.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for MLC-based gym activity recognition on STM32F401RE using the LSM6DSV16X sensor and UCF-generated configuration code.*
