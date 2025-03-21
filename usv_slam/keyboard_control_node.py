#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import sys, tty, termios, threading, time

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.get_logger().info("Keyboard Control Node started.")
        
        # GPIO Pin Definitions
        self.W_PINS = [20, 21]
        self.S_PINS = [12, 16]
        self.A_PINS = [25, 1]
        self.D_PINS = [8, 7]
        self.motor1_in1 = 24
        self.motor1_in2 = 13
        self.motor2_in3 = 18
        self.motor2_in4 = 23

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
            self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH)  # Initially set all pins HIGH

        # Key state dictionary
        self.key_states = {'w': False, 's': False, 'a': False, 'd': False, 'r': False}
        self.conveyor_running = False
        self.conveyor_thread = None

        # Start the key listener in a separate thread
        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

    def getch(self):
        """Get a single character from standard input without echoing."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def rotate_clockwise(self, duration):
        """Rotate motors clockwise for a duration."""
        GPIO.output(self.motor1_in1, GPIO.HIGH)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.HIGH)
        GPIO.output(self.motor2_in4, GPIO.LOW)
        time.sleep(duration)
        self.stop_motors()

    def rotate_anticlockwise(self, duration):
        """Rotate motors anticlockwise for a duration."""
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.HIGH)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.HIGH)
        time.sleep(duration)
        self.stop_motors()

    def stop_motors(self):
        """Stops both motors."""
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.LOW)

    def run_conveyor(self):
        """Runs the conveyor system continuously as long as enabled."""
        while self.conveyor_running:
            self.rotate_clockwise(15)
            time.sleep(1)
            self.rotate_anticlockwise(15)
            time.sleep(1)
        self.stop_motors()

    def handle_key(self, key):
        """Handles key presses and toggles corresponding GPIO actions."""
        if key in self.key_states:
            if key == 'r':
                self.key_states['r'] = not self.key_states['r']
                self.conveyor_running = self.key_states['r']
                if self.conveyor_running:
                    self.get_logger().info("Conveyor system started.")
                    self.conveyor_thread = threading.Thread(target=self.run_conveyor, daemon=True)
                    self.conveyor_thread.start()
                else:
                    self.get_logger().info("Conveyor system stopped.")
            else:
                self.key_states[key] = not self.key_states[key]
                if key == 'w':
                    pins = self.W_PINS
                elif key == 's':
                    pins = self.S_PINS
                elif key == 'a':
                    pins = self.A_PINS
                elif key == 'd':
                    pins = self.D_PINS
                else:
                    return

                state = GPIO.LOW if self.key_states[key] else GPIO.HIGH
                for pin in pins:
                    GPIO.output(pin, state)
                self.get_logger().info(f"Key '{key}' toggled. Pins {pins} set to {'LOW' if state == GPIO.LOW else 'HIGH'}.")
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt

    def key_listener(self):
        """Continuously listens for keystrokes."""
        try:
            while rclpy.ok():
                key = self.getch()
                self.handle_key(key)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard Interrupt, shutting down.")
        finally:
            self.conveyor_running = False
            if self.conveyor_thread:
                self.conveyor_thread.join()
            GPIO.cleanup()
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt in main.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
