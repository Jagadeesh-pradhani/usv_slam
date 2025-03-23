#!/usr/bin/env python3
import time
import math
import threading
import sys
import RPi.GPIO as GPIO
from smbus2 import SMBus

############################################
# MPU9250 SETTINGS & I2C REGISTERS
############################################
MPU9250_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# Scale modifiers for default settings
ACCEL_SCALE_MODIFIER_2G = 16384.0
GYRO_SCALE_MODIFIER_250DEG = 131.0

def read_i2c_word(bus, addr, reg):
    """
    Reads two bytes from the MPU9250 at 'reg' and returns a signed value.
    """
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
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
    # Set gyro config (±250 deg/s)
    bus.write_byte_data(addr, GYRO_CONFIG, 0x00)
    time.sleep(0.01)
    # Set accel config (±2g)
    bus.write_byte_data(addr, ACCEL_CONFIG, 0x00)
    time.sleep(0.01)

############################################
# MPU9250 CLASS: YAW ESTIMATION VIA GYRO INTEGRATION
############################################
class MPU9250:
    def __init__(self):
        self.bus = SMBus(1)  # Use I2C bus 1 on Raspberry Pi
        initialize_mpu(self.bus, MPU9250_ADDR)
        self.addr = MPU9250_ADDR
        # Initialize integration variables
        self.last_time = time.time()
        self.yaw = 0.0  # in radians

    def read_yaw(self):
        """
        Reads the gyroscope Z-axis, integrates over time to update yaw.
        Returns the current yaw in radians (normalized to [-pi, pi]).
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Read raw gyro Z value (offset = 4 bytes from GYRO_XOUT_H)
        gz_raw = read_i2c_word(self.bus, self.addr, GYRO_XOUT_H + 4)
        # Convert to deg/s (assuming ±250 deg/s range)
        gz_deg_s = gz_raw / GYRO_SCALE_MODIFIER_250DEG
        # Convert deg/s to rad/s
        gz_rad_s = math.radians(gz_deg_s)

        # Integrate: yaw = yaw + omega * dt
        self.yaw += gz_rad_s * dt

        # Normalize yaw to [-pi, pi)
        self.yaw = normalize_angle(self.yaw)
        return self.yaw

    def close(self):
        self.bus.close()

############################################
# GPIO PIN DEFINITIONS
############################################
W_PINS = [20, 21]     # Forward
S_PINS = [12, 16]     # Backward
A_PINS = [25, 1]      # Left
D_PINS = [8, 7]       # Right

# Conveyor Motor Pin Definitions
motor1_in1 = 24
motor1_in2 = 13
motor2_in3 = 18
motor2_in4 = 23

# All GPIO pins used
ALL_PINS = W_PINS + S_PINS + A_PINS + D_PINS + [motor1_in1, motor1_in2, motor2_in3, motor2_in4]

############################################
# PATH SEQUENCE (in cm for forward, deg for turn)
############################################
# Example mission: you can adjust dimensions as needed.
breadth = 150        # cm
length =  100        # cm

path_sequence = [
    ('forward', breadth/2),   # move forward half breadth
    ('turn',   -90),          # turn right 90°
    ('forward', length),       # move forward length
    ('turn',    90),          # turn left 90°
    ('forward', breadth/2),   # move forward half breadth
    ('turn',    90),          # turn left 90°
    ('forward', length/2),     # move forward half length
    ('turn',    90),          # turn left 90°
    ('forward', breadth),     # move forward full breadth
    ('turn',   -90),          # turn right 90°
    ('forward', length/2),     # move forward half length
    ('turn',   180),          # 180° turn
    ('forward', length),       # move forward length
    ('stop',      0),         # stop
]

FORWARD_SPEED_CMS = 20.0  # Example: 20 cm/s
TURN_SPEED_DEG_S = 30.0   # Example: 30 deg/s

RUN_CONVEYOR = True

############################################
# GPIO & MOTOR CONTROL FUNCTIONS
############################################
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in ALL_PINS:
        GPIO.setup(pin, GPIO.OUT)
        # Set pins HIGH initially (for movement pins, HIGH is "off")
        GPIO.output(pin, GPIO.HIGH)

def cleanup_gpio():
    GPIO.cleanup()

def stop_all_motors():
    # Stop movement motors
    for pin in W_PINS + S_PINS + A_PINS + D_PINS:
        GPIO.output(pin, GPIO.HIGH)
    # Stop conveyor motors
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in3, GPIO.LOW)
    GPIO.output(motor2_in4, GPIO.LOW)

def move_forward(enable: bool):
    if enable:
        # Activate forward pins (set LOW) and keep others HIGH
        for pin in W_PINS:
            GPIO.output(pin, GPIO.LOW)
        for pin in S_PINS + A_PINS + D_PINS:
            GPIO.output(pin, GPIO.HIGH)
    else:
        for pin in W_PINS:
            GPIO.output(pin, GPIO.HIGH)

def move_backward(enable: bool):
    if enable:
        for pin in S_PINS:
            GPIO.output(pin, GPIO.LOW)
        for pin in W_PINS + A_PINS + D_PINS:
            GPIO.output(pin, GPIO.HIGH)
    else:
        for pin in S_PINS:
            GPIO.output(pin, GPIO.HIGH)

def move_left(enable: bool):
    if enable:
        for pin in A_PINS:
            GPIO.output(pin, GPIO.LOW)
        for pin in W_PINS + S_PINS + D_PINS:
            GPIO.output(pin, GPIO.HIGH)
    else:
        for pin in A_PINS:
            GPIO.output(pin, GPIO.HIGH)

def move_right(enable: bool):
    if enable:
        for pin in D_PINS:
            GPIO.output(pin, GPIO.LOW)
        for pin in W_PINS + S_PINS + A_PINS:
            GPIO.output(pin, GPIO.HIGH)
    else:
        for pin in D_PINS:
            GPIO.output(pin, GPIO.HIGH)

############################################
# CONVEYOR CONTROL FUNCTIONS
############################################
def conveyor_clockwise():
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in3, GPIO.HIGH)
    GPIO.output(motor2_in4, GPIO.LOW)

def conveyor_anticlockwise():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
    GPIO.output(motor2_in3, GPIO.LOW)
    GPIO.output(motor2_in4, GPIO.HIGH)

def conveyor_stop():
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
    GPIO.output(motor2_in3, GPIO.LOW)
    GPIO.output(motor2_in4, GPIO.LOW)

def run_conveyor_forever():
    while True:
        conveyor_clockwise()
        time.sleep(15)
        conveyor_stop()
        time.sleep(1)
        conveyor_anticlockwise()
        time.sleep(15)
        conveyor_stop()
        time.sleep(1)

############################################
# HELPER MATH FUNCTIONS
############################################
def normalize_angle(angle_rad):
    angle_rad = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if angle_rad < 0:
        angle_rad += 2.0 * math.pi
    return angle_rad - math.pi

def angle_diff(target, current):
    d = target - current
    d = math.fmod(d + math.pi, 2.0 * math.pi)
    if d < 0:
        d += 2.0 * math.pi
    return d - math.pi

############################################
# MAIN PATH FOLLOWING LOGIC
############################################
def execute_path(mpu):
    """
    Execute the predefined path:
      - 'forward': move forward using a time-based command.
      - 'turn': rotate until the integrated yaw difference is reached.
      - 'stop': stop the robot.
    """
    current_yaw = normalize_angle(mpu.read_yaw())
    initial_yaw = current_yaw

    for (action, value) in path_sequence:
        if action == "stop":
            print("Stopping final step...")
            stop_all_motors()
            break

        elif action == "forward":
            distance_cm = value
            duration = distance_cm / FORWARD_SPEED_CMS
            print(f"Forward {distance_cm} cm (~{duration:.2f}s).")
            move_forward(True)
            start_time = time.time()
            while (time.time() - start_time) < duration:
                time.sleep(0.01)
            move_forward(False)
            stop_all_motors()
            # Update heading after forward movement
            current_yaw = normalize_angle(mpu.read_yaw())
            initial_yaw = current_yaw

        elif action == "turn":
            angle_deg = value
            angle_rad = math.radians(angle_deg)
            target_yaw = normalize_angle(initial_yaw + angle_rad)
            print(f"Turning {angle_deg}° to target yaw (rad): {target_yaw:.2f}")
            max_turn_time = abs(angle_deg) / TURN_SPEED_DEG_S + 2.0

            start_time = time.time()
            while True:
                current_yaw = normalize_angle(mpu.read_yaw())
                diff = angle_diff(target_yaw, current_yaw)

                if abs(diff) < math.radians(2.0):
                    break

                direction = 1 if diff > 0 else -1
                if direction > 0:
                    move_left(True)
                    move_right(False)
                else:
                    move_right(True)
                    move_left(False)

                if (time.time() - start_time) > max_turn_time:
                    print("Turn timed out, stopping.")
                    break

                time.sleep(0.01)

            move_left(False)
            move_right(False)
            stop_all_motors()
            current_yaw = normalize_angle(mpu.read_yaw())
            initial_yaw = current_yaw

        else:
            print(f"Unknown action: {action}")

    print("Path execution complete.")

############################################
# MAIN ENTRY POINT
############################################
def main():
    setup_gpio()
    stop_all_motors()  # Ensure motors are stopped at the start

    # Create MPU9250 instance (which will handle I2C and yaw integration)
    mpu = MPU9250()

    # Optionally run the conveyor in a background thread
    conveyor_thread = None
    if RUN_CONVEYOR:
        conveyor_thread = threading.Thread(target=run_conveyor_forever, daemon=True)
        conveyor_thread.start()

    try:
        execute_path(mpu)
    except KeyboardInterrupt:
        print("KeyboardInterrupt - Exiting...")
    finally:
        print("Cleaning up GPIO...")
        stop_all_motors()
        cleanup_gpio()
        mpu.close()
        sys.exit(0)

if __name__ == "__main__":
    main()
