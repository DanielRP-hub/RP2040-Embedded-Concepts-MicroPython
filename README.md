# RP2040 Embedded Concepts

## Repository Overview
This repository serves as a workspace for rapid algorithmic prototyping and hardware validation utilizing the Raspberry Pi Pico (ARM Cortex-M0+ / RP2040). The primary focus is the implementation of complex mathematical models, sensor fusion algorithms, and digital signal processing routines.

While production-level automotive firmware is strictly developed in bare-metal C or C++, utilizing MicroPython in this repository allows for agile iteration of mathematical concepts, immediate Hardware-in-the-Loop (HIL) testing, and rapid data visualization before optimizing the final logic into lower-level languages.

## Hardware Architecture
* Microcontroller: Raspberry Pi Pico (RP2040 Dual-Core ARM Cortex-M0+ @ 133MHz)
* Primary Sensors: MPU6050 (6-axis IMU)
* Communication Interfaces: I2C (Fast Mode 400kHz), Asynchronous UART for CLI

## Project Directory

### 1. Advanced IMU Sensor Fusion (AHRS)
A high-fidelity Attitude and Heading Reference System prototype. This module focuses on eliminating MEMS sensor drift and noise through advanced numerical methods.
* Technical Highlights: 
  * Implementation of Zero Velocity Updates (ZUPT) for dynamic gyroscope bias compensation.
  * Integration of Welford's Online Algorithm to compute real-time variance and establish dynamic deadbands.
  * Utilization of Simpson's 3/8 Rule for higher-order numerical integration of the Yaw angle, significantly reducing the truncation errors typical of standard Euler integration.

### 2. Digital Inclinometer via Accelerometry
A highly responsive 2-axis tilt sensor that isolates the static 1g gravity vector to calculate precise Pitch and Roll angles.
* Technical Highlights:
  * Hardware-level optimization using Python's `struct.unpack` for single-cycle Big-Endian byte parsing of I2C burst reads.
  * Implementation of a non-blocking UART Command Line Interface (CLI) using `select.select()` for real-time dynamic offset calibration without halting the main telemetry execution loop.
