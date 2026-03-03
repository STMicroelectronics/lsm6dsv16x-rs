# Examples for LSM6DSV16X Sensor

This project abstracts board-specific details, enabling easy selection and setup of supported boards and frameworks while keeping examples consistent.

## Supported Boards and Frameworks

| Board-id               | stm32-rs Framework | Embassy Framework |
|------------------------|--------------------|-------------------|
| nucleo-f401re          | ✓                  |                   |
| nucleo-f401re-embassy  |                    | ✓                 |

## Quickstart

To run an example, first select the appropriate board-id from the table above. Then execute the example with Cargo by specifying the example as a feature.

### Example command syntax:

```sh
cargo <board_id> --features <example_name>
```

### Example usage

Run the *read_irq* example on the Nucleo F401RE using the stm32-rs framework:
```sh
cargo nucleo-f401re --features read_irq
```

Run the same example using the Embassy async framework:
```sh
cargo nucleo-f401re-embassy --features read_irq
```

## Board Configuration
Board-specific configurations such as pins for I2C (SDA, SCL), UART, and interrupt lines are abstracted via macros in the BSP source code. These macros simplify adapting examples to different hardware setups.

### Configuration Macros in bsp/src/main.rs
For the **Embassy** framework (**nucleo-f401re-embassy** board-id):
```rust
define_embassy_with_st_link! {
    i2c = {
        address: I2CAddress::I2cAddH as u8,
        periph: I2C1,
        scl: PB8,
        sda: PB9,
        ev_irq: I2C1_EV,
        er_irq: I2C1_ER,
        dma_tx: DMA1_CH7,
        dma_rx: DMA1_CH0,
    },
    uart = {
        periph: USART2,
        tx: PA2,
        dma_tx: DMA1_CH6,
        baud: 115200,
    },
    int_pin = {
        pin: PB4,
        exti_line: EXTI4,
        exti_mux: EXTI4,
    }
}
```
For the **stm32-rs** HAL framework (**nucleo-f401re** board-id):
```rust
#[cfg(feature = "nucleo-f401re")]
define_stm32_rs_with_st_link!(
    i2c = {
        address: I2CAddress::I2cAddH as u8,
        periph: I2C1,
        scl: (port_b, pb8),
        sda: (port_b, pb9),
    },
    uart = {
        periph: USART2,
        tx: (port_a, pa2),
    },
    interrupt = {
        pin: (port_b, pb4),
        exti_irq: EXTI4,
    }
);
```
*Note*:

- The sensor I2C address can be changed depending on the SD0 pin state.
- This configuration is tested using the sensor built into the IKS4A01 shield.

## Available Examples
Examples are located in the [src/examples](src/examples) directory. Below is a list of available examples:

- compressed_fifo.rs — Example demonstrating compressed FIFO usage.
- fifo.rs — FIFO buffer handling.
- fifo_irq.rs — FIFO with interrupt-driven data reading.
- fifo_stepcnt.rs — Step counter using FIFO.
- free_fall.rs — Free fall detection example.
- fsm_fourd.rs — Finite State Machine example for four-direction detection.
- fsm_glance.rs — FSM example for glance detection.
- mlc_gym.rs — Machine Learning Core example for gym activity recognition.
- pedometer.rs — Step counting example.
- qvar.rs — Qvar user interface example.
- read_irq.rs — Data reading using interrupts.
- read_polling.rs — Data reading using polling.
- self_test.rs — Sensor self-test example.
- sensor_fusion.rs — Sensor fusion example.
- sensor_hub.rs — Sensor hub example.
- sig_mot.rs — Significant motion detection example.
- single_double_tap.rs — Tap detection example.
- sixd_position.rs — 6D position detection.
- tilt.rs — Tilt detection.
- wakeup.rs — Wakeup event detection.

## FSM and MLC Configuration Files

Some examples use configuration files for the Finite State Machine (FSM) and Machine Learning Core (MLC). These JSON files are located in [src/config](src/config):

- lsm6dsv16x_fourd_orientation.json
- lsm6dsv16x_glance.json
- lsm6dsv16x_gym_activity_recognition_right.json

These configuration files are automatically included and converted to Rust code for examples that require them, via the [build.rs](build.rs) build script.
