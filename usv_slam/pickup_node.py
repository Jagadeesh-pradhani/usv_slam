#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class PickupNode(Node):
    def __init__(self):
        super().__init__('pickup_node')
        self.get_logger().info("Pickup Node started.")
        
        # Pin Definitions
        self.motor1_in1 = 14  
        self.motor1_in2 = 24  
        self.motor2_in3 = 18  
        self.motor2_in4 = 23 

        GPIO.setmode(GPIO.BCM)
        for pin in [self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]:
            GPIO.setup(pin, GPIO.OUT)
        
        # Create a timer to call the rotation function repeatedly
        self.timer = self.create_timer(0.1, self.rotate_anticlockwise)

    def rotate_anticlockwise(self):
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.HIGH)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.HIGH)

    def stop_motors(self):
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.LOW)

    def destroy_node(self):
        self.stop_motors()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PickupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down Pickup Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
