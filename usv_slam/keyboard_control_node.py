#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gpiod
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
        
        # GPIO Setup using updated gpiod API
        self.chip = gpiod.Chip('gpiochip0')
        
        # Create dictionaries to store lines
        self.pins = {}
        
        # Configure all pins
        for pin in self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
            self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]:
            try:
                # Get the GPIO line and configure it as output
                self.pins[pin] = self.chip.get_line(pin)
                self.pins[pin].request(consumer="keyboard_control", type=gpiod.LINE_REQ_DIR_OUT)
                self.pins[pin].set_value(1)  # Initially set all pins HIGH
            except Exception as e:
                self.get_logger().error(f"Error setting up pin {pin}: {e}")

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

    def set_pin_value(self, pin, value):
        """Set a pin value safely."""
        try:
            if pin in self.pins:
                self.pins[pin].set_value(value)
            else:
                self.get_logger().error(f"Pin {pin} not configured")
        except Exception as e:
            self.get_logger().error(f"Error setting pin {pin} to {value}: {e}")

    def rotate_clockwise(self, duration):
        """Rotate motors clockwise for a duration."""
        self.set_pin_value(self.motor1_in1, 1)
        self.set_pin_value(self.motor1_in2, 0)
        self.set_pin_value(self.motor2_in3, 1)
        self.set_pin_value(self.motor2_in4, 0)
        time.sleep(duration)
        self.stop_motors()

    def rotate_anticlockwise(self, duration):
        """Rotate motors anticlockwise for a duration."""
        self.set_pin_value(self.motor1_in1, 0)
        self.set_pin_value(self.motor1_in2, 1)
        self.set_pin_value(self.motor2_in3, 0)
        self.set_pin_value(self.motor2_in4, 1)
        time.sleep(duration)
        self.stop_motors()

    def stop_motors(self):
        """Stops both motors."""
        self.set_pin_value(self.motor1_in1, 0)
        self.set_pin_value(self.motor1_in2, 0)
        self.set_pin_value(self.motor2_in3, 0)
        self.set_pin_value(self.motor2_in4, 0)

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

                state = 0 if self.key_states[key] else 1  # 0 = LOW, 1 = HIGH
                for pin in pins:
                    self.set_pin_value(pin, state)
                self.get_logger().info(f"Key '{key}' toggled. Pins {pins} set to {'LOW' if state == 0 else 'HIGH'}.")
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
            # Release GPIO resources
            for pin in self.pins.values():
                pin.release()
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