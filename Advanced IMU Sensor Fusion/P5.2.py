"""
Advanced IMU Sensor Fusion and Integration via I2C.
Rapid prototyping of yaw estimation algorithms using the RP2040 and MPU6050.
Implements Simpson's 3/8 integration, dynamic deadbands (Welford's algorithm), 
and Zero Velocity Updates (ZUPT) for gyro bias compensation.

Author: Daniel Ruiz Perez
"""

from machine import Pin, I2C
import utime as time
import math

# ====== Hardware Configuration ======
# I2C0 Pins: SDA=GP4, SCL=GP5
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400_000)

# Hardware interrupt button for Yaw reset
BTN_PIN = 14
btn = Pin(BTN_PIN, Pin.IN, Pin.PULL_UP)
_last_btn = 1

# ====== MPU6050 Registers ======
MPU_ADDR          = 0x68
REG_PWR_MGMT_1    = 0x6B
REG_SMPLRT_DIV    = 0x19
REG_CONFIG        = 0x1A
REG_GYRO_CONFIG   = 0x1B
REG_ACCEL_CONFIG  = 0x1C
REG_ACCEL_XOUT_H  = 0x3B

# ====== Operational Parameters ======
SAMPLE_HZ   = 100.0
DT          = 1.0 / SAMPLE_HZ
SMPLRT_DIV  = 9                 # 1kHz/(1+9) = 100 Hz (with active DLPF)
DLPF_5HZ    = 6                 # Digital Low Pass Filter ~5 Hz
GYRO_FS_SEL = 0b10              # 0b10 = ±1000 dps (32.8 LSB/dps)

# ====== Scaling Factors ======
GYRO_SENS = 32.8                # LSB/(deg/s) for ±1000 dps
ACC_SENS  = 16384.0             # LSB/g for ±2g

# ====== Calibration & Bias ======
CALIB_SAMPLES = 500
gyroBiasZ = 0.0                

# ====== Simpson 3/8 Integration State ======
wZ = [0.0, 0.0, 0.0, 0.0]
idx = 0
stepCount = 0
thetaGz = 0.0
thetaGz_prev = 0.0
primed = False

# ====== ZUPT (Zero Velocity Update) & Hysteresis ======
OMEGA_ENTER_STILL = 0.6    
OMEGA_EXIT_STILL  = 1.0    
ACC_NORM_THR      = 0.05   
TAU_BIAS_Z        = 2.0    
K_BIAS_Z          = 1.0 / TAU_BIAS_Z
K_BIAS_Z_FAST     = 1.0 / 0.5  
still = False
still_count = 0            

# ====== Prefilter (Median of 3) & Spike Handling ======
gz_hist = [0.0, 0.0, 0.0]
gz_hist_i = 0
gz_lpf = 0.0
FC_HZ = 3.0                                         
ALPHA_LPF = 1.0 - math.exp(-2.0 * math.pi * FC_HZ * DT)
SPIKE_THR_DPS = 50.0                                

# ====== Adaptive Deadband (Welford's Algorithm) ======
noise_n = 0
noise_mean = 0.0
noise_M2 = 0.0

DEADBAND_K = 3.0             
Z_DEADBAND_DPS_MIN = 0.03
Z_DEADBAND_DPS_MAX = 0.20
Z_DEADBAND_DPS_FIXED = 0.06  
Z_DEADBAND_DPS_DYNAMIC = Z_DEADBAND_DPS_FIXED

yaw_deg = 0.0

# ====== Hardware Interfaces ======

def mpuWrite(reg, val):
    """Writes a single byte to a specific MPU6050 register."""
    i2c.writeto_mem(MPU_ADDR, reg, bytes([val]))

def mpuReadN(startReg, n):
    """Reads N sequential bytes from the MPU6050."""
    return i2c.readfrom_mem(MPU_ADDR, startReg, n)

def s16(hi, lo):
    """Converts two 8-bit registers into a signed 16-bit integer."""
    v = (hi << 8) | lo
    if v & 0x8000:
        v -= 0x10000
    return v

# ====== Mathematical Utilities ======

def absSum3(x, y, z):
    return abs(x) + abs(y) + abs(z)

def median3(a, b, c):
    """Calculates the median of three values to filter out high-frequency anomalies."""
    if a > b:
        if b > c: return b
        return a if a < c else c
    else:
        if a > c: return a
        return b if b < c else c

def welford_update(x):
    """
    Online computation of sample variance using Welford's algorithm.
    Dynamically adjusts the integration deadband based on real-time noise floors.
    """
    global noise_n, noise_mean, noise_M2
    noise_n += 1
    delta = x - noise_mean
    noise_mean += delta / noise_n
    noise_M2 += delta * (x - noise_mean)

def noise_sigma():
    """Returns the standard deviation of the tracked noise."""
    return math.sqrt(noise_M2 / (noise_n - 1)) if noise_n > 1 else 0.0

# ====== Core Algorithmic Functions ======

def setupMPU():
    """Initializes the MPU6050 with specific power, filter, and scale settings."""
    mpuWrite(REG_PWR_MGMT_1, 0x00)           
    time.sleep_ms(100)
    mpuWrite(REG_CONFIG, DLPF_5HZ & 0x07)    
    mpuWrite(REG_GYRO_CONFIG, (GYRO_FS_SEL & 0x03) << 3)
    mpuWrite(REG_ACCEL_CONFIG, 0x00)         
    mpuWrite(REG_SMPLRT_DIV, SMPLRT_DIV)     

def readMPU():
    """Fetches raw 6-axis data (Accelerometer & Gyroscope) from the IMU."""
    buf = mpuReadN(REG_ACCEL_XOUT_H, 14)
    ax = s16(buf[0],  buf[1])
    ay = s16(buf[2],  buf[3])
    az = s16(buf[4],  buf[5])
    gx = s16(buf[8],  buf[9])
    gy = s16(buf[10], buf[11])
    gz = s16(buf[12], buf[13])
    return ax, ay, az, gx, gy, gz

def calibrateGyroZ():
    """Samples the Z-axis gyroscope at rest to establish an initial hardware bias."""
    global gyroBiasZ
    sz = 0
    time.sleep_ms(500)
    for _ in range(CALIB_SAMPLES):
        _, _, _, _, _, gz = readMPU()
        sz += gz
        time.sleep_ms(2)
    gyroBiasZ = (sz / float(CALIB_SAMPLES)) / GYRO_SENS  

def integrateSimpson38Z(wz):
    """
    Performs highly accurate numerical integration using Simpson's 3/8 rule.
    Provides superior area-under-the-curve estimation compared to standard Euler integration.
    """
    global thetaGz, idx, stepCount, primed
    prev = (idx + 3) & 3  

    if primed:
        thetaGz += 0.5 * DT * (wZ[prev] + wz)  

    wZ[idx] = wz
    idx = (idx + 1) & 3
    stepCount += 1
    if (not primed) and stepCount >= 2:
        primed = True

    if stepCount >= 4 and ((stepCount - 1) % 3 == 0):
        i0 = (idx + 0) & 3
        i1 = (idx + 1) & 3
        i2 = (idx + 2) & 3
        i3 = (idx + 3) & 3

        S = (3.0 * DT / 8.0) * (wZ[i0] + 3*wZ[i1] + 3*wZ[i2] + wZ[i3])
        T = 0.5 * DT * (wZ[i0] + wZ[i1]) \
          + 0.5 * DT * (wZ[i1] + wZ[i2]) \
          + 0.5 * DT * (wZ[i2] + wZ[i3])
        thetaGz += (S - T)

# ====== Main Execution Loop ======

def setup():
    setupMPU()
    calibrateGyroZ()
    print("MPU6050 Init: Fs~100Hz, DLPF=5Hz")

def loop():
    global yaw_deg, thetaGz, thetaGz_prev, gyroBiasZ, still, _last_btn
    global gz_hist, gz_hist_i, gz_lpf, Z_DEADBAND_DPS_DYNAMIC, still_count

    period_us = int(DT * 1_000_000)
    next_t = time.ticks_us()

    while True:
        next_t = time.ticks_add(next_t, period_us)

        # 1. Hardware Polling & Scaling
        axr, ayr, azr, gxr, gyr, gzr = readMPU()
        ax = axr / ACC_SENS
        ay = ayr / ACC_SENS
        az = azr / ACC_SENS
        gx = gxr / GYRO_SENS
        gy = gyr / GYRO_SENS
        gz_s = gzr / GYRO_SENS            

        # 2. Prefiltering (Anti-spike clamp & Median Filter)
        if stepCount > 0:
            prev_raw = gz_hist[(gz_hist_i - 1) % 3]
            if abs(gz_s - prev_raw) > SPIKE_THR_DPS:
                gz_s = prev_raw  

        gz_hist[gz_hist_i] = gz_s
        gz_hist_i = (gz_hist_i + 1) % 3
        gz_med = median3(gz_hist[0], gz_hist[1], gz_hist[2])
        gz_lpf = gz_lpf + ALPHA_LPF * (gz_med - gz_lpf)

        # 3. ZUPT (Zero Velocity Update) Detection
        omega_norm = absSum3(gx, gy, (gz_lpf - gyroBiasZ))  
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)

        if not still:
            if (omega_norm < OMEGA_ENTER_STILL) and (abs(a_norm - 1.0) < ACC_NORM_THR):
                still = True
                still_count = 0
        else:
            if (omega_norm > OMEGA_EXIT_STILL) or (abs(a_norm - 1.0) >= ACC_NORM_THR):
                still = False

        # 4. Adaptive Bias Correction & Noise Profiling
        if still:
            k = K_BIAS_Z_FAST if still_count < int(0.5 / DT) else K_BIAS_Z
            gyroBiasZ += (DT * k) * (gz_lpf - gyroBiasZ)  
            still_count += 1

            welford_update(gz_lpf - gyroBiasZ)
            if noise_n >= 100:  
                sig = noise_sigma()
                Z_DEADBAND_DPS_DYNAMIC = min(
                    Z_DEADBAND_DPS_MAX,
                    max(Z_DEADBAND_DPS_MIN, DEADBAND_K * sig)
                )

        # 5. Signal Conditioning
        gz = gz_lpf - gyroBiasZ
        if abs(gz) < Z_DEADBAND_DPS_DYNAMIC:
            gz = 0.0

        # 6. Advanced Numerical Integration (Simpson 3/8)
        thetaGz_prev = thetaGz
        integrateSimpson38Z(gz)
        dtheta = (thetaGz - thetaGz_prev)
        yaw_deg = (yaw_deg + dtheta) % 360.0

        # 7. Hardware HMI (Reset)
        btn_val = btn.value()
        if (_last_btn == 1) and (btn_val == 0):
            yaw_deg = 0.0
        _last_btn = btn_val

        # 8. Telemetry Output
        print("Yaw:{:.2f} | gz:{:.2f} | biasZ:{:.4f} | dband:{:.3f}".format(
            yaw_deg, gz, gyroBiasZ, Z_DEADBAND_DPS_DYNAMIC
        ))

        # 9. Strict Timing Enforcement
        now = time.ticks_us()
        remain = time.ticks_diff(next_t, now)
        if remain > 0:
            time.sleep_us(remain)

if __name__ == "__main__":
    try:
        def setupMPU_safe():
            try:
                setupMPU()
            except Exception as e:
                time.sleep_ms(100)
                setupMPU()

        setupMPU_safe()
        setup()
        loop()
    except KeyboardInterrupt:
        print("System Halted.")