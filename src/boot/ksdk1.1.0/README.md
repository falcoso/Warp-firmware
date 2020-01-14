## Overview
This fork of the Warp Firmware repository modifies the original firmware to create a controller for an inverted pendulum system.

The core of the firmware is in `warp-kl03-ksdk1.1-boot.c`. The drivers for the individual sensors are in `devXXX.c` for sensor `XXX`. For example,
`devMMA8451Q.c` for the MMA8451Q 3-axis accelerometer.

## Macros

## `LOOP_READINGS`
Defined in `warp-kl03-ksdk1.1-boot.c`, this macro compiles the firmware to start up in the inverted pendulum control loop.

## `PRINT_F`
Defined in `PID.h` if this is enabled along with `LOOP_READINGS`, after initialising the robot will wait for an input from the terminal. Once any key is pressed the robot will start the controller and output angle readings, and if the INA219 is connected, current readings as well, in `.csv` format.

## Not defining `LOOP_READINGS`
This will compile the robot in the original Warp Firmware menu with additional options to configure PWM. NOTE modifications to this were made in early stages of development so some menu options may present unexpected behavior and is only included for debugging purposes.

## Source File Descriptions

##### `CMakeLists.txt`
This is the CMake configuration file. Edit this to change the default size of the stack and heap.


##### `SEGGER_RTT.*`
This is the implementation of the SEGGER Real-Time Terminal interface. Do not modify.

##### `SEGGER_RTT_Conf.h`
Configuration file for SEGGER Real-Time Terminal interface. You can increase the size of `BUFFER_SIZE_UP` to reduce text in the menu being trimmed.

##### `SEGGER_RTT_printf.c`
Implementation of the SEGGER Real-Time Terminal interface formatted I/O routines. Do not modify.

##### `devINA219.*`
Driver for INA219.

##### `devL289N.*`
Driver for L289N. TPM and LPTMR variables are initialised in here for Pule Width Modulation generation on the motor driver circuits.

##### `devMMA8451Q.*`
Driver for MMA8451Q.

##### `devMPU6050.*`
Driver for MPU6050.

##### `devSSD1331.*`
Driver for SSD1331.

##### `gpio_pins.c`
Definition of I/O pin configurations using the KSDK `gpio_output_pin_user_config_t` structure.

##### `gpio_pins.h`
Definition of I/O pin mappings and aliases for different I/O pins to symbolic names relevant to the Warp hardware design, via `GPIO_MAKE_PIN()`.

##### `startup_MKL03Z4.S`
Initialization assembler.

##### `PID.*`
Library for creating a PID controller based on MPU6050 readings.

##### `warp-kl03-ksdk1.1-boot.c`
The core of the implementation. This puts together the processor initialization with a menu interface that triggers the individual sensor drivers based on commands entered at the menu, or loads the robot to control an inverted pendulum depending on the macros used. In any mode, after initialisation the LED will light up to say that the robot is ready.

##### `warp-kl03-ksdk1.1-powermodes.c`
Implements functionality related to enabling the different low-power modes of the KL03.

##### `warp.h`
Constant and data structure definitions.
