# Bluetooth_Car_Project

This project implements a Bluetooth-controlled system using a TM4C123 launchpad ARM Cortex M4 microcontroller. The system controls a DC motor and LEDs based on Bluetooth commands. It supports various movements (forward, backward, left, right) and PWM speed control for the motor. Additionally, it has an onboard LED feedback mechanism.

## Features

- **PWM Motor Control**: Controls two DC motors (left and right) with adjustable speeds.
- **Direction Control**: Move the motors forward, backward, left, or right.
- **Speed Control**: Adjust the motor speed in multiple increments using Bluetooth commands.
- **LED Feedback**: RGB LED feedback to show the current system status.
- **Bluetooth Interface**: Receives commands from a Bluetooth device to control the motors and LEDs.
- **Interrupt Handling**: Uses GPIO interrupts to handle button presses (SW1, SW2) for additional control.

## Hardware Requirements

- **TM4C123 microcontroller**: This code is written for the TM4C123GH6PM microcontroller.
- **Bluetooth Module**: The system communicates with a Bluetooth module via UART.
- **DC Motors**: Two motors controlled via PWM for speed and direction.
- **RGB LED**: Feedback LED that changes colors based on system states.
- **Push Buttons**: SW1 and SW2 for additional control.

## Connections

- **Motor Control**:
  - Motor 1 (Left Motor): Controlled via PWM0A (PB6).
  - Motor 2 (Right Motor): Controlled via PWM0B (PB7).
  - Direction control: PE0, PE1 for the right motor, PE2, PE3 for the left motor.
- **LED Feedback**:
  - RGB LED: Connected to GPIO Port F (PF1, PF2, PF3).
- **Push Buttons**: Connected to GPIO Port F (PF0 for SW1, PF4 for SW2).
- **Bluetooth Module**: UART0 for communication with a Bluetooth module (Rx/Tx).

## Software Overview

The software is structured to receive Bluetooth commands through UART, interpret them, and control the motors accordingly. The system has the following key functionality:

1. **Direction Control**: The system can move the motors in different directions:
   - Forward (F/f)
   - Backward (B/b)
   - Left (L/l)
   - Right (R/r)
   
2. **Speed Control**: The speed of the motors can be adjusted by sending the following commands:
   - Increase speed (U/u)
   - Decrease speed (D/d)
   
3. **Stopping the Motors**: Send the `S/s` command to stop both motors.

4. **Special Movement (C/c)**: A special motion where the system moves forward for a brief moment, adjusts the speed, and then stops.

5. **PWM Control**: The system uses PWM signals to control the motor speeds. The duty cycle can be adjusted to change the motor speed.

6. **Button Press Handling**: When a button is pressed, it generates an interrupt, and simple debouncing is implemented to handle it.

## Bluetooth Control Commands

The system receives and executes the following Bluetooth commands:

| Command | Action                            |
|---------|-----------------------------------|
| `F`     | Move Forward                      |
| `B`     | Move Backward                     |
| `L`     | Turn Left                         |
| `R`     | Turn Right                        |
| `S`     | Stop the motors                   |
| `U`     | Increase speed                    |
| `D`     | Decrease speed                    |
| `C`     | Special movement (forward with speed change) |

## Setup Instructions

### Hardware Setup

1. **Connect the Motors** to the appropriate PWM pins (PB6 for left motor and PB7 for right motor).
2. **Connect the RGB LED** to GPIO Port F (PF1, PF2, PF3).
3. **Connect the Push Buttons** to GPIO Port F (SW1 to PF0, SW2 to PF4).
4. **Connect the Bluetooth Module** to UART0 on the TM4C123 microcontroller.

### Software Setup

1. Clone this repository to your local machine.
2. Import the project into your IDE (e.g., Keil, Code Composer Studio).
3. Build and upload the code to your TM4C123 microcontroller.
4. Pair your Bluetooth device (e.g., smartphone) with the microcontroller and use a terminal app to send commands.

## Interrupts and Debouncing

The system uses GPIO interrupts to handle button presses. The interrupt handler includes a simple delay loop for debouncing, ensuring that the button press is processed correctly.


