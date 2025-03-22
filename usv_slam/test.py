#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gpiozero import LED, Device
from subprocess import run
import sys, tty, termios, threading, time, select, os
from subprocess import run, PIPE

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

        # Reset all GPIO pins before initializing
        self.reset_all_gpio_pins()

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

        # GPIO Setup using gpiozero:
        # Create a dictionary to store LED objects.
        # Using active_high=False so that off() means HIGH (inactive) and on() means LOW (active).
        self.pins = {}
        all_pins = self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
            self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
        for pin in all_pins:
            try:
                # Initialize each pin to inactive (off -> HIGH)
                self.pins[pin] = LED(pin, active_high=False, initial_value=False)
            except Exception as e:
                self.get_logger().error(f"Error setting up pin {pin}: {e}")

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

    def reset_all_gpio_pins(self):
        """Reset all GPIO pins at startup to ensure they're not busy"""
        self.get_logger().info("Resetting all GPIO pins...")
        try:
            # Get all pins that need to be reset
            all_pins = self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
                self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
            
            # Method 1: Try using gpio command-line tool if available
            try:
                for pin in all_pins:
                    run(["gpio", "-g", "mode", str(pin), "in"], check=False, stderr=PIPE)
            except Exception as e:
                self.get_logger().debug(f"gpio command failed: {e}")
            
            # Method 2: Try releasing pins through sysfs
            for pin in all_pins:
                try:
                    # Check if pin is exported
                    if os.path.exists(f"/sys/class/gpio/gpio{pin}"):
                        # Try to unexport the pin
                        with open("/sys/class/gpio/unexport", "w") as f:
                            f.write(str(pin))
                except Exception as e:
                    self.get_logger().debug(f"Could not unexport pin {pin}: {e}")
            
            # Method 3: Try using RPi.GPIO if available
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                for pin in all_pins:
                    try:
                        GPIO.cleanup(pin)
                    except:
                        pass
            except ImportError:
                self.get_logger().debug("RPi.GPIO not available")
            
            # Sleep to allow changes to take effect
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Error resetting GPIO pins: {e}")
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
        """
        Sets the pin value.
          - value == 0 -> Active (LOW) => call on()
          - value == 1 -> Inactive (HIGH) => call off()
        """
        try:
            if pin in self.pins:
                if value == 0:
                    self.pins[pin].on()
                else:
                    self.pins[pin].off()
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
                if key == 'x':  # Force stop: reset movement keys
                    for k in ['w', 's', 'a', 'd']:
                        self.key_states[k] = False
                        pins = self.get_pins_for_key(k)
                        for pin in pins:
                            try:
                                self.set_pin_value(pin, 1)  # 1 = inactive (HIGH)
                            except Exception as e:
                                self.get_logger().error(f"Error setting pin for key {k}: {e}")
                    print("FORCE STOP")
                else:
                    # Toggle the key state for movement keys
                    self.key_states[key] = not self.key_states[key]
                    pins = self.get_pins_for_key(key)
                    
                    try:
                        state = 0 if self.key_states[key] else 1  # 0 = active (LOW), 1 = inactive (HIGH)
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
            # Release GPIO resources by closing all LED devices
            try:
                for pin in self.pins.values():
                    try:
                        pin.close()
                    except:
                        pass
                # Try RPi.GPIO cleanup as well
                try:
                    import RPi.GPIO as GPIO
                    GPIO.cleanup()
                except:
                    pass
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
        # Ensure all resources are properly cleaned up
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()