"""
Digital Inclinometer and Tilt Sensing via Accelerometry.
Rapid prototyping of a 2-axis tilt sensor using the RP2040 and MPU6050.
Features dynamic non-blocking UART calibration and fast I2C struct unpacking.

Author: Daniel Ruiz Perez
"""

from machine import Pin, I2C
import time
import math
import sys
import select
import struct

# ===================== MPU6050 Registers =====================
MPU6050_CONFIG        = 0x1A
MPU6050_GYRO_CONFIG   = 0x1B
MPU6050_ACCEL_CONFIG  = 0x1C
MPU6050_PWR_MGMT_1    = 0x6B
MPU6050_ACCEL_XOUT_H  = 0x3B
MPU6050_ADDRESS       = 0x68  

# ===================== Physical Constants =====================
ACC_SENS_LSB_PER_G = 16384.0   # Scale factor for ±2g range
G_SI               = 9.80665   # Standard gravity in m/s^2
CAL_SAMPLES        = 1000      # Number of samples for dynamic averaging

# ===================== Hardware Offsets =====================
off_ax = 0
off_ay = 0
off_az = 0

# ===================== Hardware Interfaces =====================
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)

def write_reg(reg, val):
    """Writes a single byte to the specified I2C register."""
    i2c.writeto_mem(MPU6050_ADDRESS, reg, bytes([val]))
    time.sleep_us(20)

def read_burst_14():
    """Reads 14 sequential bytes starting from ACCEL_XOUT_H."""
    return i2c.readfrom_mem(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14)

def rad2deg(r):
    """Converts radians to degrees."""
    return r * 57.295775

def read_raw():
    """
    Fetches and unpacks 14 bytes of raw sensor data.
    Utilizes struct.unpack for highly efficient big-endian byte joining.
    Returns: ax, ay, az, gx, gy, gz, temp (Signed 16-bit integers).
    """
    data = read_burst_14()
    # ">" specifies Big-Endian, "hhhhhhh" specifies 7 signed 16-bit shorts
    ax, ay, az, temp, gx, gy, gz = struct.unpack(">hhhhhhh", data)
    return ax, ay, az, gx, gy, gz, temp

# ===================== Dynamic Calibration Routines =====================

def calibrar_z_pos():
    """Calibrates the Z-axis by averaging static 1g forces."""
    global off_az
    print("Calibrating Z+ (Keep sensor flat, Z pointing up).")
    time.sleep_ms(500)
    saz = 0
    for _ in range(CAL_SAMPLES):
        ax, ay, az, gx, gy, gz, t = read_raw()
        saz += az
        time.sleep_ms(2)
    mz = saz / CAL_SAMPLES
    off_az = int(mz - 16384.0)  # Target is +1g (+16384 LSB)
    print("Done. off_az =", off_az)

def calibrar_x_pos():
    """Calibrates the X-axis by averaging static 1g forces."""
    global off_ax
    print("Calibrating X+ (Keep sensor sideways, X pointing down).")
    time.sleep_ms(500)
    sax = 0
    for _ in range(CAL_SAMPLES):
        ax, ay, az, gx, gy, gz, t = read_raw()
        sax += ax
        time.sleep_ms(2)
    mx = sax / CAL_SAMPLES
    off_ax = int(mx - 16384.0)  
    print("Done. off_ax =", off_ax)

def calibrar_y_pos():
    """Calibrates the Y-axis by averaging static 1g forces."""
    global off_ay
    print("Calibrating Y+ (Keep sensor on its edge, Y pointing down).")
    time.sleep_ms(500)
    say = 0
    for _ in range(CAL_SAMPLES):
        ax, ay, az, gx, gy, gz, t = read_raw()
        say += ay
        time.sleep_ms(2)
    my = say / CAL_SAMPLES
    off_ay = int(my - 16384.0)  
    print("Done. off_ay =", off_ay)

def mpu6050_init():
    """Configures the IMU with strong low-pass filtering for static tilt sensing."""
    # DLPF ~5 Hz (Heavy filtering for gravity isolation), Gyro ignored, Accel ±2 g
    write_reg(MPU6050_CONFIG,       0x06)
    write_reg(MPU6050_GYRO_CONFIG,  0x10)
    write_reg(MPU6050_ACCEL_CONFIG, 0x00)
    write_reg(MPU6050_PWR_MGMT_1,   0x00)
    time.sleep_ms(50)

# ===================== Main Execution =====================

def main():
    print("MPU-6050 Inclinometer Ready.")
    print("UART Calibration Commands:")
    print("  'z' -> Calibrate Z-Axis (+1g)")
    print("  'x' -> Calibrate X-Axis (+1g)")
    print("  'y' -> Calibrate Y-Axis (+1g)\n")
    time.sleep_ms(1000)

    while True:
        # Non-blocking UART polling for CLI commands
        if select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch in ('z', 'Z'):
                calibrar_z_pos()
            elif ch in ('x', 'X'):
                calibrar_x_pos()
            elif ch in ('y', 'Y'):
                calibrar_y_pos()

        # 1. Hardware Polling
        ax, ay, az, gx, gy, gz, temp = read_raw()

        # 2. Offset Application
        ax -= off_ax
        ay -= off_ay
        az -= off_az

        # 3. Scale to G-Force
        ax_g = ax / ACC_SENS_LSB_PER_G
        ay_g = ay / ACC_SENS_LSB_PER_G
        az_g = az / ACC_SENS_LSB_PER_G

        # 4. Trigonometric Tilt Calculation (Roll & Pitch)
        # Note: True Yaw cannot be calculated from a stationary accelerometer.
        roll_deg  = rad2deg(math.atan2(ax_g, az_g))
        pitch_deg = rad2deg(math.atan2(ay_g, az_g))

        if roll_deg  < 0:  roll_deg  += 360.0
        if pitch_deg < 0:  pitch_deg += 360.0

        # 5. Physics Conversion (m/s^2)
        ax_ms2 = ax_g * G_SI
        ay_ms2 = ay_g * G_SI
        az_ms2 = az_g * G_SI

        # 6. Telemetry Output
        print("ACC [m/s^2] x:{0:.2f}, y:{1:.2f}, z:{2:.2f} | Tilt [deg] Roll(X):{3:.2f}, Pitch(Y):{4:.2f}".format(
            ax_ms2, ay_ms2, az_ms2, roll_deg, pitch_deg))

        time.sleep_ms(200)

def _whoami_check():
    """Verifies I2C connectivity and device address."""
    try:
        _ = i2c.readfrom_mem(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 1)
        return True
    except OSError as e:
        print("I2C Bus Error. Check wiring or I2C Address (0x68/0x69).")
        return False

if __name__ == "__main__":
    if _whoami_check():
        mpu6050_init()
        main()