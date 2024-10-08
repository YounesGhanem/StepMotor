# STM32 Stepper Motor Control Firmware

This repository contains firmware for controlling stepper motors using the L6470 motor driver and STM32F401RE-Nucleo. The firmware allows for motor control via USART commands.

## Features

- **Motor Control via USART:** Control stepper motors using simple serial commands.
- **Flash Memory Storage:** Save and restore motor positions across power cycles.
- **Dual Operating Modes:** Choose between basic motor control and USART-controlled motor control.
- **Error Handling:** Comprehensive command validation and error reporting.

## Important Notes / Limitations

The hardware used in this project does not include any position sensors (such as a Hall sensor or encoder). This limitation means the software cannot determine its exact starting point after a power cycle or reset. It only tracks relative movements from an arbitrary starting position.

When the motor's position is saved to flash, it stores the position relative to this unknown starting point. Upon reset or power cycle, the motor will start again from an arbitrary position, rendering the saved position data effectively meaningless without an absolute reference.

To make saved positions useful, the system would need to establish a "home" or zero position upon startup using a position sensor. This would provide a known reference point, allowing the system to restore the motor to an exact known position after a reset.

## Hardware Requirements

- **STM32 Nucleo Board:** STM32F401RE, ready to use by default.
- **X-NUCLEO-IHM02A1:** Dual stepper motor driver expansion board.
- **Stepper Motors:** 2 x Stepper motors compatible with the L6470 driver.

## Software Requirements

- **VS Code:** For building and debugging the firmware.
- **ST-Link and OpenOCD:** The code is developed for use with ST-Link, OpenOCD, and GDB for debugging and flashing. A dedicated paper on the IDE and toolchain will be provided.

## Target Configuration

The code is configured for a specific STM32 target using ST-Link and OpenOCD. If you need to change the target board or MCU, ensure you select the appropriate configuration files. These files should be placed in the `openocd_configs` directory within the project.

## Getting Started

### Command Examples

The motors can only be commanded via USART. Below are examples of commands. Please note that the values are in microsteps.

- `M1.RUN.FWD.200` - Motor M1 runs forward at speed 200.
- `M0.MOV.REV.500` - Motor M0 moves in reverse 500 steps.
- `M1.GOTO.FWD.1000` - Motor M1 goes to position 1000 forward.
- `M0.SOFTSTOP` - Stops Motor M0 immediately.

### TODO

- Connect a GUI for a more user-friendly interface.

### Cloning the Repository

```bash
git clone https://github.com/YounesGhanem/StepMotor.git
cd STM32-Stepper-Motor-Control
