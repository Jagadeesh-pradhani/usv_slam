#!/usr/bin/env python3

import time
import math
from smbus2 import SMBus
import json

MPU9250_ADDR = 0x68
PWR_MGMT_1   = 0x6B
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ACCEL_SCALE_MODIFIER_2G = 16384.0
GYRO_SCALE_MODIFIER_250DEG = 131.0

def read_i2c_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def initialize_mpu(bus, addr):
    bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
    time.sleep(0.01)
    bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
    time.sleep(0.01)

def read_accel_gyro(bus, addr):
    ax_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H)
    ay_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H + 2)
    az_raw = read_i2c_word(bus, addr, ACCEL_XOUT_H + 4)

    gx_raw = read_i2c_word(bus, addr, GYRO_XOUT_H)
    gy_raw = read_i2c_word(bus, addr, GYRO_XOUT_H + 2)
    gz_raw = read_i2c_word(bus, addr, GYRO_XOUT_H + 4)

    ax = ax_raw / ACCEL_SCALE_MODIFIER_2G
    ay = ay_raw / ACCEL_SCALE_MODIFIER_2G
    az = az_raw / ACCEL_SCALE_MODIFIER_2G

    gx = gx_raw / GYRO_SCALE_MODIFIER_250DEG
    gy = gy_raw / GYRO_SCALE_MODIFIER_250DEG
    gz = gz_raw / GYRO_SCALE_MODIFIER_250DEG

    return (ax, ay, az, gx, gy, gz)

def main():
    bus = SMBus(1)
    initialize_mpu(bus, MPU9250_ADDR)

    num_samples = 1000
    accel_sum = [0.0, 0.0, 0.0]
    gyro_sum  = [0.0, 0.0, 0.0]

    print(f"Place the MPU9250 flat and still. Collecting {num_samples} samples...")
    time.sleep(2.0)

    for i in range(num_samples):
        ax, ay, az, gx, gy, gz = read_accel_gyro(bus, MPU9250_ADDR)
        accel_sum[0] += ax
        accel_sum[1] += ay
        accel_sum[2] += az
        gyro_sum[0]  += gx
        gyro_sum[1]  += gy
        gyro_sum[2]  += gz
        time.sleep(0.002)  # ~500 Hz sample

    # Compute average
    accel_avg = [a / num_samples for a in accel_sum]
    gyro_avg  = [g / num_samples for g in gyro_sum]

    print("Average Accelerometer (g):", accel_avg)
    print("Average Gyroscope (deg/s):", gyro_avg)

    # Typically, we want the offsets so that:
    #   ax_offset = -accel_avg[0]
    #   ay_offset = -accel_avg[1]
    #   az_offset = 1.0 - accel_avg[2]  (assuming Z should read ~ +1g)
    #   gx_offset = -gyro_avg[0]
    #   ...
    # This depends on your orientation. If you want raw offsets so that the
    # raw reading plus offset => 0 g in X, 0 g in Y, +1 g in Z, 0 deg/s in gyro:
    ax_offset = -accel_avg[0]
    ay_offset = -accel_avg[1]
    az_offset = 1.0 - accel_avg[2]  # if you want Z to be +1g
    gx_offset = -gyro_avg[0]
    gy_offset = -gyro_avg[1]
    gz_offset = -gyro_avg[2]

    offsets = {
        "ax_offset": ax_offset,
        "ay_offset": ay_offset,
        "az_offset": az_offset,
        "gx_offset": gx_offset,
        "gy_offset": gy_offset,
        "gz_offset": gz_offset
    }

    print("Computed Offsets:", offsets)

    # Save to a file
    with open("mpu_offsets.txt", "w") as f:
        json.dump(offsets, f, indent=2)

    print("Offsets saved to mpu_offsets.txt")

    bus.close()

if __name__ == "__main__":
    main()
