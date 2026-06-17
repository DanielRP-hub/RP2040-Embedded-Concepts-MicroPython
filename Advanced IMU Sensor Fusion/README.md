## Overview
This repository contains a MicroPython-based prototype for advanced Attitude and Heading Reference System (AHRS) algorithms running on the Raspberry Pi Pico (RP2040). The primary objective of this project is to rapidly validate complex sensor fusion and noise filtering mathematics using real-world I2C data from an MPU6050 before translating the logic into optimized bare-metal C/C++.

## Hardware Architecture
* Microcontroller: Raspberry Pi Pico (ARM Cortex-M0+ / RP2040)
* Sensor: MPU6050 6-axis IMU (I2C at 400kHz)
* Timing: Strict 100Hz execution loop utilizing hardware microsecond ticks (`time.ticks_us()`) to prevent integration drift.

## Advanced Mathematical Implementations

### 1. Zero Velocity Update (ZUPT)
To mitigate the inherent drift of MEMS gyroscopes, this firmware implements a ZUPT algorithm. It continuously monitors the sum of angular rates and acceleration vector norms to detect true physical stillness. When the system is static, it aggressively recalculates the `gyroBiasZ` offset, effectively eliminating long-term drift.

### 2. Adaptive Deadbands via Welford's Algorithm
Instead of using a hardcoded threshold to ignore background sensor noise, the system employs Welford's Online Algorithm to calculate the running variance and standard deviation of the sensor noise during static periods. The integration deadband is dynamically adjusted based on real-time hardware noise floors, preventing micro-vibrations from accumulating into angular drift.

### 3. Simpson's 3/8 Rule for Numerical Integration
Standard Euler integration (multiplying angular velocity by time delta) introduces significant truncation errors over time. This prototype utilizes Simpson's 3/8 Rule, a higher-order numerical method that approximates the integration area using cubic interpolation, yielding a drastically higher fidelity Yaw angle estimation.

### 4. Multi-Stage Prefiltering
Before integration, raw data passes through a multi-stage software conditioning pipeline:
* Spike Clamping: Ignores physical shocks exceeding predefined limits.
* Median Filter (Window = 3): Removes high-frequency outliers.
* Infinite Impulse Response (IIR) Low Pass Filter: Smooths the signal with a software cutoff frequency of ~3Hz.
