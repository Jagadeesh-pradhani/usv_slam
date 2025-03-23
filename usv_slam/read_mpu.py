#!/usr/bin/env python3

import time
import math
from smbus2 import SMBus

# Common I2C address for MPU9250 (check yours!)
MPU9250_ADDR = 0x68

# MPU-9250 internal registers (for accelerometer & gyro)
# Refer to the MPU-9250 Register Map for details
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# Scale modifiers from datasheet (for default +/- 2g, +/- 250 deg/s)
ACCEL_SCALE_MODIFIER_2G = 16384.0
GYRO_SCALE_MODIFIER_250DEG = 131.0

def read_i2c_word(bus, addr, reg):
    """
    Reads two bytes from the MPU9250 at 'reg' and returns a signed value.
    """
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    # Convert to signed
    if value > 32767:
        value -= 65536
    return value

def initialize_mpu(bus, addr):
    """
    Wakes up the MPU9250 and configures basic settings.
    """
    # Wake up the MPU9250 (clear sleep bit)
    bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # Optional: Set gyro config (±250 deg/s)
    bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
    time.sleep(0.01)

    # Optional: Set accel config (±2g)
    bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
    time.sleep(0.01)

def read_accel_gyro(bus, addr):
    """
    Returns (ax, ay, az, gx, gy, gz)
    in units of (g, g, g, deg/s, deg/s, deg/s)
    """
    # Read accelerometer raw data
    ax_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H)
    ay_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H + 2)
    az_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H + 4)

    # Read gyroscope raw data
    gx_raw = read_i2c_word(bus, addr, GYRO_XOUT_H)
    gy_raw = read_i2c_word(bus, addr, GYRO_XOUT_H + 2)
    gz_raw = read_i2c_word(bus, addr, GYRO_XOUT_H + 4)

    # Convert to 'g' for accelerometer
    ax = ax_raw / ACCEL_SCALE_MODIFIER_2G
    ay = ay_raw / ACCEL_SCALE_MODIFIER_2G
    az = az_raw / ACCEL_SCALE_MODIFIER_2G

    # Convert to deg/s for gyroscope
    gx = gx_raw / GYRO_SCALE_MODIFIER_250DEG
    gy = gy_raw / GYRO_SCALE_MODIFIER_250DEG
    gz = gz_raw / GYRO_SCALE_MODIFIER_250DEG

    return (ax, ay, az, gx, gy, gz)

def main():
    bus = SMBus(1)  # Use I2C bus 1 on Raspberry Pi
    initialize_mpu(bus, MPU9250_ADDR)

    print("Reading MPU9250 data. Press Ctrl+C to stop.")
    try:
        while True:
            ax, ay, az, gx, gy, gz = read_accel_gyro(bus, MPU9250_ADDR)
            print(f"Accel (g): x={ax:.3f}, y={ay:.3f}, z={az:.3f} | "
                  f"Gyro (deg/s): x={gx:.3f}, y={gy:.3f}, z={gz:.3f}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        bus.close()

if __name__ == "__main__":
    main()
