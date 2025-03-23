#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import math
from smbus2 import SMBus

############################################
# GPIO PIN DEFINITIONS (example for turning)
############################################
# For turning right, we use D_PINS from your setup.
D_PINS = [8, 7]

# Motor pin definitions for the conveyor system (not used here)
motor1_in1 = 24
motor1_in2 = 13
motor2_in3 = 18
motor2_in4 = 23

# For this test, we only need the turning pins.
TURN_PINS = D_PINS

############################################
# MPU9250 I2C Setup and Register Definitions
############################################
MPU9250_ADDR = 0x68
PWR_MGMT_1   = 0x6B
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
GYRO_XOUT_H  = 0x43  # Gyro readings start here

# Scale modifier for ±250°/s (default)
GYRO_SCALE_MODIFIER_250DEG = 131.0

def read_i2c_word(bus, addr, reg):
    """Read two bytes from a given register and return signed word."""
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def initialize_mpu(bus, addr):
    """Wake up and configure the MPU9250."""
    # Wake up the MPU9250 (clear sleep bit)
    bus.write_byte_data(addr, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # Set gyro config to ±250 deg/s (0x00 default)
    bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
    time.sleep(0.01)
    # Optionally, you can configure the accelerometer as well
    bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
    time.sleep(0.01)

def read_gyro_z(bus, addr):
    """
    Reads the gyroscope Z-axis data in deg/s.
    This provides the angular speed.
    """
    gz_raw = read_i2c_word(bus, addr, GYRO_XOUT_H + 4)  # Z is offset by 4 from X
    gz = gz_raw / GYRO_SCALE_MODIFIER_250DEG
    return gz

############################################
# GPIO and Motor Functions for Turning
############################################
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    # Setup turning pins
    for pin in TURN_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)  # HIGH means "off" for directional control

def cleanup_gpio():
    GPIO.cleanup()

def move_right(enable: bool):
    """
    Enables turning to the right by setting the right-turn pins.
    When enable is True, set pins to LOW to activate turning.
    When enable is False, set pins to HIGH to stop.
    """
    state = GPIO.LOW if enable else GPIO.HIGH
    for pin in D_PINS:
        GPIO.output(pin, state)

############################################
# MAIN TEST FUNCTION: TURNING WITH IMU FEEDBACK
############################################
def main():
    # Parameters for the test
    turn_duration = 5.0    # seconds: how long to command the turn
    sample_interval = 0.1  # seconds: interval for printing IMU readings

    # Setup GPIO and MPU
    setup_gpio()
    bus = SMBus(1)  # Use I2C bus 1 on Raspberry Pi
    initialize_mpu(bus, MPU9250_ADDR)
    
    print("Starting turning test. Commanding a right turn for {} seconds.".format(turn_duration))
    
    # Start turning: For a right turn, we call move_right(True)
    move_right(True)
    start_time = time.time()
    prev_time = start_time

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time
            if elapsed > turn_duration:
                break
            
            # Read current angular speed (deg/s) from the IMU's gyro Z-axis.
            gyro_z = read_gyro_z(bus, MPU9250_ADDR)
            
            # Optionally, you can calculate delta time and integrate the angle.
            dt = current_time - prev_time
            prev_time = current_time

            print("Time: {:.2f}s | Angular Speed (gyro Z): {:.2f} deg/s".format(elapsed, gyro_z))
            time.sleep(sample_interval)
    
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    
    finally:
        # Stop turning command and clean up
        move_right(False)
        cleanup_gpio()
        bus.close()
        print("Test completed. Motors stopped and GPIO cleaned up.")

if __name__ == "__main__":
    main()
