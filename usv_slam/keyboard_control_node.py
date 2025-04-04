#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import gpiod
import sys, tty, termios, threading, time, select, os

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

        # Control message
        self.msg = """
Control Your Bot with Keyboard!
---------------------------
Moving around:
        w
   a    s    d
        x

w : move forward
s : move backward
a : turn left
d : turn right
r : toggle conveyor system
x : force stop

CTRL-C to quit
"""

        # GPIO Setup using gpiod
        try:
            self.chip = gpiod.Chip('gpiochip0')
            # Create dictionaries to store lines
            self.pins = {}
            
            # Configure all pins
            for pin in self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
                self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]:
                try:
                    # Get the GPIO line and configure it as output
                    line = self.chip.get_line(pin)
                    line.request(consumer="keyboard_control", type=gpiod.LINE_REQ_DIR_OUT)
                    line.set_value(1)  # Initially set all pins HIGH (inactive)
                    self.pins[pin] = line
                except Exception as e:
                    self.get_logger().error(f"Error setting up pin {pin}: {e}")
        except Exception as e:
            self.get_logger().error(f"GPIO Setup Error: {e}")
            self.get_logger().info("Continuing with limited functionality - GPIO pins may not work.")

        # Key state dictionary
        self.key_states = {'w': False, 's': False, 'a': False, 'd': False, 'r': False}
        self.conveyor_running = False
        self.conveyor_thread = None
        self.status = 0

        # Print the instructions
        print(self.msg)

        # Start the key listener in a separate thread
        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

    def get_key(self):
        """Get a single character from standard input without echoing."""
        if os.name == 'nt':
            import msvcrt
            return msvcrt.getch().decode('utf-8')
        else:
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                else:
                    key = ''
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return key

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
        try:
            self.set_pin_value(self.motor1_in1, 1)
            self.set_pin_value(self.motor1_in2, 0)
            self.set_pin_value(self.motor2_in3, 1)
            self.set_pin_value(self.motor2_in4, 0)
            time.sleep(duration)
            self.stop_motors()
        except Exception as e:
            self.get_logger().error(f"Error in rotate_clockwise: {e}")

    def rotate_anticlockwise(self, duration):
        """Rotate motors anticlockwise for a duration."""
        try:
            self.set_pin_value(self.motor1_in1, 0)
            self.set_pin_value(self.motor1_in2, 1)
            self.set_pin_value(self.motor2_in3, 0)
            self.set_pin_value(self.motor2_in4, 1)
            time.sleep(duration)
            self.stop_motors()
        except Exception as e:
            self.get_logger().error(f"Error in rotate_anticlockwise: {e}")

    def stop_motors(self):
        """Stops both motors."""
        try:
            self.set_pin_value(self.motor1_in1, 0)
            self.set_pin_value(self.motor1_in2, 0)
            self.set_pin_value(self.motor2_in3, 0)
            self.set_pin_value(self.motor2_in4, 0)
        except Exception as e:
            self.get_logger().error(f"Error in stop_motors: {e}")

    def run_conveyor(self):
        """Runs the conveyor system continuously as long as enabled."""
        while self.conveyor_running:
            self.rotate_clockwise(15)
            time.sleep(1)
            self.rotate_anticlockwise(15)
            time.sleep(1)
        self.stop_motors()

    def print_directions(self):
        """Prints current active directions."""
        directions = []
        if self.key_states['w']:
            directions.append("FORWARD")
        if self.key_states['s']:
            directions.append("BACKWARD")
        if self.key_states['a']:
            directions.append("LEFT")
        if self.key_states['d']:
            directions.append("RIGHT")
        if self.key_states['r']:
            directions.append("CONVEYOR ON")
        
        if directions:
            print("Currently active:", " + ".join(directions))
        else:
            print("Currently: STOPPED")

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
                # Reset other movement keys if 'x' is pressed
                if key == 'x':
                    for k in ['w', 's', 'a', 'd']:
                        self.key_states[k] = False
                        pins = self.get_pins_for_key(k)
                        for pin in pins:
                            try:
                                self.set_pin_value(pin, 1)  # HIGH = inactive
                            except Exception as e:
                                self.get_logger().error(f"Error setting pin for key {k}: {e}")
                    print("FORCE STOP")
                else:
                    # Toggle the key state
                    self.key_states[key] = not self.key_states[key]
                    pins = self.get_pins_for_key(key)
                    
                    try:
                        state = 0 if self.key_states[key] else 1  # 0 = LOW (active), 1 = HIGH (inactive)
                        for pin in pins:
                            self.set_pin_value(pin, state)
                    except Exception as e:
                        self.get_logger().error(f"Error setting pin for key {key}: {e}")
                    
                    self.status += 1
            
            # Print directions after any key press
            self.print_directions()
            
            # Periodically show the full menu
            if self.status == 20:
                print(self.msg)
                self.status = 0
                
        elif key == '\x03':  # Ctrl+C
            raise KeyboardInterrupt

    def get_pins_for_key(self, key):
        """Returns the pins associated with a key."""
        if key == 'w':
            return self.W_PINS
        elif key == 's':
            return self.S_PINS
        elif key == 'a':
            return self.A_PINS
        elif key == 'd':
            return self.D_PINS
        else:
            return []

    def key_listener(self):
        """Continuously listens for keystrokes."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:  # Only process if a key was actually pressed
                    self.handle_key(key)
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard Interrupt, shutting down.")
        finally:
            self.conveyor_running = False
            if self.conveyor_thread:
                self.conveyor_thread.join()
            # Release GPIO resources
            try:
                for pin in self.pins.values():
                    pin.release()
            except Exception as e:
                self.get_logger().error(f"Error during GPIO cleanup: {e}")
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