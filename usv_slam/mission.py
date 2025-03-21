#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ----- GPIO PIN ASSIGNMENTS -----
# Movement Motor Pins (for autonomous driving)
W_PINS = [20, 21]  # Forward
S_PINS = [12, 16]  # Backward
A_PINS = [25, 1]   # Left turn (if needed)
D_PINS = [8, 7]    # Right turn (if needed)

# Pickup/Drop Mechanism Motor Pins
PICKUP_MOTOR1_IN1 = 14  
PICKUP_MOTOR1_IN2 = 24  
PICKUP_MOTOR2_IN3 = 18  
PICKUP_MOTOR2_IN4 = 23 

# ----- INITIALIZE GPIO -----
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup movement motor pins (initial state HIGH = motors off)
for pin in (W_PINS + S_PINS + A_PINS + D_PINS):
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.HIGH)

# Setup pickup/drop motor pins (initial state LOW)
for pin in [PICKUP_MOTOR1_IN1, PICKUP_MOTOR1_IN2, PICKUP_MOTOR2_IN3, PICKUP_MOTOR2_IN4]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# ----- MISSION NODE -----
class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info("Mission Node started. Running fully autonomous pick and place mission.")
        
        # Create subscribers for camera and LIDAR
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        # State machine variables
        self.state = "SEARCH"   # Other states: PICKUP, MOVE_TO_DROP, DROP
        self.ball_detected = False
        self.detected_ball_color = None  # Valid: 'blue', 'green', 'yellow'
        self.last_lidar_distance = None
        
        # Valid ball colors (red is not valid in this example)
        self.valid_colors = ['blue', 'green', 'yellow']
        
        # Define HSV ranges for ball colors (may need calibration)
        self.color_ranges = {
            'red':    ((0, 100, 100), (10, 255, 255)),
            'blue':   ((100, 150, 0), (140, 255, 255)),
            'green':  ((40, 70, 70), (80, 255, 255)),
            'yellow': ((20, 100, 100), (30, 255, 255))
        }
        
        # Create a timer for the state machine (runs every 1 second)
        self.timer = self.create_timer(1.0, self.state_machine_callback)

    # --------------------
    # CAMERA IMAGE CALLBACK
    # --------------------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        ball_found = False
        
        # Loop through valid colors only
        for color in self.valid_colors:
            lower, upper = self.color_ranges[color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Filter noise by area threshold
                    # Draw bounding rectangle for visualization
                    (x, y, w, h) = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    self.detected_ball_color = color
                    ball_found = True
                    break
            if ball_found:
                break
        
        self.ball_detected = ball_found
        
        # Optionally, display the image (for debugging)
        cv2.imshow("Ball Detection", cv_image)
        cv2.waitKey(1)

    # --------------------
    # LIDAR SCAN CALLBACK
    # --------------------
    def lidar_callback(self, msg):
        if msg.ranges:
            self.last_lidar_distance = min(msg.ranges)
        else:
            self.last_lidar_distance = None

    # --------------------
    # STATE MACHINE CALLBACK
    # --------------------
    def state_machine_callback(self):
        # Log current state
        self.get_logger().info(f"State: {self.state}")
        
        if self.state == "SEARCH":
            # In search state, if a valid ball is detected, transition to PICKUP.
            if self.ball_detected:
                self.get_logger().info(f"Valid ball detected: {self.detected_ball_color}. Initiating pickup.")
                self.state = "PICKUP"
            else:
                self.get_logger().info("Searching for valid ball...")
                
        elif self.state == "PICKUP":
            # Activate pickup mechanism for a fixed duration
            self.get_logger().info("Activating pickup mechanism...")
            self.pickup_ball(duration=3)
            # After pickup, transition to MOVE_TO_DROP state
            self.state = "MOVE_TO_DROP"
            
        elif self.state == "MOVE_TO_DROP":
            # Move forward until drop-off location is detected using LIDAR
            if self.last_lidar_distance is not None:
                self.get_logger().info(f"LIDAR distance: {self.last_lidar_distance:.2f} m")
                if self.last_lidar_distance < 0.5:
                    self.get_logger().info("Drop-off location reached.")
                    self.stop_movement()
                    self.state = "DROP"
                else:
                    self.get_logger().info("Moving towards drop-off point...")
                    self.move_forward(duration=0.5)
            else:
                self.get_logger().warn("No LIDAR data available. Cannot determine drop-off location.")
                
        elif self.state == "DROP":
            # Activate drop mechanism for a fixed duration
            self.get_logger().info("Activating drop mechanism...")
            self.drop_ball(duration=3)
            self.get_logger().info("Ball dropped off. Resuming search.")
            # Reset ball detection flag and go back to search state
            self.ball_detected = False
            self.detected_ball_color = None
            self.state = "SEARCH"
            
    # --------------------
    # MOTOR CONTROL FUNCTIONS
    # --------------------
    def move_forward(self, duration):
        # For movement, set the forward motor pins (W_PINS) active (LOW) and others inactive (HIGH)
        for pin in W_PINS:
            GPIO.output(pin, GPIO.LOW)
        for pin in S_PINS + A_PINS + D_PINS:
            GPIO.output(pin, GPIO.HIGH)
        time.sleep(duration)
        self.stop_movement()

    def stop_movement(self):
        for pin in (W_PINS + S_PINS + A_PINS + D_PINS):
            GPIO.output(pin, GPIO.HIGH)

    def pickup_ball(self, duration):
        # Activate the pickup motors in anticlockwise direction
        GPIO.output(PICKUP_MOTOR1_IN1, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR1_IN2, GPIO.HIGH)
        GPIO.output(PICKUP_MOTOR2_IN3, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR2_IN4, GPIO.HIGH)
        time.sleep(duration)
        self.stop_pickup_motors()

    def drop_ball(self, duration):
        # Activate the pickup motors in clockwise direction (simulate drop-off)
        GPIO.output(PICKUP_MOTOR1_IN1, GPIO.HIGH)
        GPIO.output(PICKUP_MOTOR1_IN2, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR2_IN3, GPIO.HIGH)
        GPIO.output(PICKUP_MOTOR2_IN4, GPIO.LOW)
        time.sleep(duration)
        self.stop_pickup_motors()

    def stop_pickup_motors(self):
        # Turn off pickup/drop motors
        GPIO.output(PICKUP_MOTOR1_IN1, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR1_IN2, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR2_IN3, GPIO.LOW)
        GPIO.output(PICKUP_MOTOR2_IN4, GPIO.LOW)

    # --------------------
    # CLEANUP ON SHUTDOWN
    # --------------------
    def destroy_node(self):
        cv2.destroyAllWindows()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mission interrupted by keyboard.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
