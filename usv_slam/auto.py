#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

class BallDetectionNode(Node):
    def __init__(self):
        super().__init__('ball_detection_node')
        self.get_logger().info("Ball Detection Node started.")
        
        # Subscribe to the raw image topic.
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Define HSV ranges for the allowed colored balls (do not detect red).
        # The order here is arbitrary since they all share the same priority.
        self.color_ranges = {
            'blue': ((100, 150, 0), (140, 255, 255)),
            'green': ((40, 70, 70), (80, 255, 255)),
            'yellow': ((20, 100, 100), (30, 255, 255))
        }

        # Motor pin definitions (adjust as needed for your hardware wiring).
        self.motor1_in1 = 24
        self.motor1_in2 = 13
        self.motor2_in3 = 18
        self.motor2_in4 = 23

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        motor_pins = [self.motor1_in1, self.motor1_in2, self.motor2_in3, self.motor2_in4]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # Control parameters
        self.drive_duration = 0.5   # seconds to drive per command
        self.center_threshold = 20  # pixel threshold to decide turning

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        ball_centers = []  # List to hold centers of detected allowed balls

        # Loop through only allowed colors.
        for color, (lower, upper) in self.color_ranges.items():
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            mask = cv2.inRange(hsv, lower_np, upper_np)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Filter out small objects/noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x = x + w / 2
                    center_y = y + h / 2
                    ball_centers.append((center_x, center_y))
                    # Draw bounding rectangle for visualization.
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if ball_centers:
            # Calculate the average x coordinate from all detected balls.
            avg_x = sum(center[0] for center in ball_centers) / len(ball_centers)
            image_center = cv_image.shape[1] / 2

            if avg_x < image_center - self.center_threshold:
                self.get_logger().info("Ball detected on the left. Turning left.")
                self.turn_left(self.drive_duration)
            elif avg_x > image_center + self.center_threshold:
                self.get_logger().info("Ball detected on the right. Turning right.")
                self.turn_right(self.drive_duration)
            else:
                self.get_logger().info("Ball detected in the center. Moving forward.")
                self.move_forward(self.drive_duration)
        else:
            self.get_logger().info("No allowed balls detected. Stopping motors.")
            self.stop_motors()

        # Display the processed image for debugging.
        cv2.imshow("Ball Detection", cv_image)
        cv2.waitKey(1)

    # Motor control functions based on your drive logic.
    def move_forward(self, duration):
        # Both motors drive forward.
        GPIO.output(self.motor1_in1, GPIO.HIGH)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.HIGH)
        GPIO.output(self.motor2_in4, GPIO.LOW)
        time.sleep(duration)
        self.stop_motors()

    def turn_left(self, duration):
        # Example: slow or stop the left motor, drive right motor forward.
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.HIGH)
        GPIO.output(self.motor2_in4, GPIO.LOW)
        time.sleep(duration)
        self.stop_motors()

    def turn_right(self, duration):
        # Example: drive left motor forward, stop the right motor.
        GPIO.output(self.motor1_in1, GPIO.HIGH)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.LOW)
        time.sleep(duration)
        self.stop_motors()

    def stop_motors(self):
        GPIO.output(self.motor1_in1, GPIO.LOW)
        GPIO.output(self.motor1_in2, GPIO.LOW)
        GPIO.output(self.motor2_in3, GPIO.LOW)
        GPIO.output(self.motor2_in4, GPIO.LOW)

    def destroy_node(self):
        cv2.destroyAllWindows()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down Ball Detection Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
