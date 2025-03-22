#!/usr/bin/env python3

import mmap
import struct
import os
import time
import sys, tty, termios, threading, select
import rclpy
from rclpy.node import Node

class MemMappedGPIO:
    """Class for direct memory-mapped access to GPIO pins."""
    
    # BCM2835 GPIO registers (Raspberry Pi)
    GPIO_BASE = 0x3F200000  # Base address for RPi 2/3, use 0x20200000 for RPi 1
    
    # Register offsets
    GPFSEL0   = 0x00  # GPIO Function Select 0
    GPSET0    = 0x1C  # GPIO Pin Output Set 0
    GPCLR0    = 0x28  # GPIO Pin Output Clear 0
    GPLEV0    = 0x34  # GPIO Pin Level 0
    
    # Direction
    GPIO_IN  = 0
    GPIO_OUT = 1
    
    def __init__(self):
        # Open /dev/mem
        if not os.path.exists("/dev/mem"):
            raise IOError("/dev/mem not found - are you running as root?")
        
        self.mem_fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        if self.mem_fd < 0:
            raise IOError("Failed to open /dev/mem")
        
        # Memory map GPIO registers
        self.gpio_map = mmap.mmap(
            self.mem_fd, 
            4096,  # Map 4KB
            mmap.MAP_SHARED, 
            mmap.PROT_READ | mmap.PROT_WRITE,
            offset=self.GPIO_BASE
        )
        
        # Check if mapping succeeded
        if not self.gpio_map:
            os.close(self.mem_fd)
            raise IOError("mmap of GPIO registers failed")
    
    def __del__(self):
        if hasattr(self, 'gpio_map') and self.gpio_map:
            self.gpio_map.close()
        if hasattr(self, 'mem_fd') and self.mem_fd >= 0:
            os.close(self.mem_fd)
    
    def _read_register(self, reg_offset):
        """Read a 32-bit register."""
        self.gpio_map.seek(reg_offset)
        return struct.unpack("<L", self.gpio_map.read(4))[0]
    
    def _write_register(self, reg_offset, value):
        """Write a 32-bit register."""
        self.gpio_map.seek(reg_offset)
        self.gpio_map.write(struct.pack("<L", value))
    
    def setup(self, pin, direction):
        """Set a pin as input or output."""
        # Calculate GPFSEL register and bit position
        reg_offset = self.GPFSEL0 + (pin // 10) * 4
        shift = (pin % 10) * 3
        
        # Read current value
        value = self._read_register(reg_offset)
        
        # Clear the 3 bits for this pin
        mask = ~(0b111 << shift)
        value &= mask
        
        # Set direction bits (001 for output, 000 for input)
        if direction == self.GPIO_OUT:
            value |= (0b001 << shift)
        
        # Write back
        self._write_register(reg_offset, value)
    
    def output(self, pin, state):
        """Set a pin output state (high/low)."""
        if state:
            reg_offset = self.GPSET0 + (pin // 32) * 4
        else:
            reg_offset = self.GPCLR0 + (pin // 32) * 4
        
        bit = 1 << (pin % 32)
        self._write_register(reg_offset, bit)
    
    def input(self, pin):
        """Read a pin state."""
        reg_offset = self.GPLEV0 + (pin // 32) * 4
        value = self._read_register(reg_offset)
        return (value & (1 << (pin % 32))) != 0
    
    def reset_pin(self, pin):
        """Reset a pin to input mode (safest state)."""
        # Set as input
        self.setup(pin, self.GPIO_IN)
        
        # For safety, also clear output register
        reg_offset = self.GPCLR0 + (pin // 32) * 4
        bit = 1 << (pin % 32)
        self._write_register(reg_offset, bit)


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

        # Initialize direct GPIO access
        try:
            self.gpio = MemMappedGPIO()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize memory-mapped GPIO: {e}")
            self.get_logger().error("Are you running this as root? Try: sudo python3 your_script.py")
            sys.exit(1)
            
        # Dictionary to track pin states (True for active/on, False for inactive/off)
        self.pin_states = {}
        
        # Initialize all pins as outputs with inactive state
        all_pins = self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
            self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
            
        for pin in all_pins:
            try:
                # Setup pin as output
                self.gpio.setup(pin, MemMappedGPIO.GPIO_OUT)
                # Set initial value to inactive (HIGH)
                self.gpio.output(pin, True)
                # Track the pin state
                self.pin_states[pin] = False  # inactive
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
        """Reset all GPIO pins at startup using direct memory access."""
        self.get_logger().info("Resetting all GPIO pins...")
        try:
            # We'll reset the problematic pins directly via memory mapping
            gpio = MemMappedGPIO()
            all_pins = self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
                self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
            
            for pin in all_pins:
                try:
                    gpio.reset_pin(pin)
                    self.get_logger().info(f"Reset pin {pin}")
                except Exception as e:
                    self.get_logger().error(f"Error resetting pin {pin}: {e}")
            
            # Clean up
            del gpio
            time.sleep(0.5)
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
          - value == 0 -> Active (LOW)
          - value == 1 -> Inactive (HIGH)
        """
        try:
            # For memory-mapped GPIO, we need to invert the value
            # value=0 (active/LOW) means output=False
            # value=1 (inactive/HIGH) means output=True
            output_value = (value == 1)
            self.gpio.output(pin, output_value)
            self.pin_states[pin] = not output_value  # Track active state (opposite of output)
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
            # Clean shutdown
            try:
                all_pins = self.W_PINS + self.S_PINS + self.A_PINS + self.D_PINS + [
                    self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
                
                for pin in all_pins:
                    try:
                        # Reset all pins to input mode (safest state)
                        self.gpio.reset_pin(pin)
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()