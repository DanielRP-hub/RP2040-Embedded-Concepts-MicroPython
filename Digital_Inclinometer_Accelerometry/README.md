## Overview
This repository contains a MicroPython-based prototype for a highly responsive 2-axis digital inclinometer utilizing the Raspberry Pi Pico (RP2040) and an MPU6050. By isolating the gravity vector from the raw accelerometer data, the system calculates precise Pitch and Roll angles. It features a non-blocking UART command-line interface (CLI) for dynamic hardware-in-the-loop (HIL) calibration.

## Technical Implementation

### 1. High-Efficiency I2C Parsing
Rather than reading and shifting high/low bytes individually, this firmware leverages Python's `struct.unpack(">hhhhhhh")` method. This allows for the immediate, single-line conversion of a 14-byte I2C burst read into seven 16-bit signed integers (Big-Endian format), drastically reducing CPU overhead during hardware polling.

### 2. Non-Blocking UART CLI
The main execution loop implements `select.select([sys.stdin])` to create an asynchronous command listener. This allows an engineer to send calibration commands (e.g., 'z', 'x', 'y') via the serial terminal to trigger dynamic offset calculations on the fly, without halting or blocking the continuous telemetry output.

### 3. Trigonometric Tilt Isolation
To act as a pure inclinometer, the IMU is configured with a heavy Digital Low Pass Filter (DLPF ~5 Hz) to reject high-frequency vibrations and dynamic accelerations, isolating the static 1g gravity vector. The orientation angles (Pitch and Roll) are then derived using the arctangent function (`math.atan2`) mapped to the respective gravitational projections on the X, Y, and Z axes. 

*(Note: True Yaw/Heading is physically unobservable by a stationary accelerometer and is thus omitted from this implementation).*
