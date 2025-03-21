#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BallDetectionNode(Node):
    def __init__(self):
        super().__init__('ball_detection_node')
        self.get_logger().info("Ball Detection Node started.")
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # Define HSV ranges for the colored balls.
        # These values are examples and may need calibration.
        self.color_ranges = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'blue': ((100, 150, 0), (140, 255, 255)),
            'green': ((40, 70, 70), (80, 255, 255)),
            'yellow': ((20, 100, 100), (30, 255, 255))
        }

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Convert BGR image to HSV for color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detected_balls = {}

        # Loop over defined colors and detect objects
        for color, (lower, upper) in self.color_ranges.items():
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            mask = cv2.inRange(hsv, lower_np, upper_np)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Filter out small objects/noise
                    (x, y, w, h) = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    detected_balls[color] = detected_balls.get(color, 0) + 1

        if detected_balls:
            self.get_logger().info(f"Detected balls: {detected_balls}")
        else:
            self.get_logger().info("No balls detected.")

        # Display the processed image (useful for debugging)
        cv2.imshow("Ball Detection", cv_image)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
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
