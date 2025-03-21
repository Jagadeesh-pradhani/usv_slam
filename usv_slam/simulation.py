#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info("Autonomous Mission Node started (simulation mode).")
        
        # Subscriptions and publisher
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State machine setup:
        # Valid states: SEARCH, APPROACH, PICKUP, MOVE_TO_DROP, DROP
        self.state = "SEARCH"
        self.state_start_time = self.get_clock().now()

        # Ball detection data
        self.ball_detected = False
        self.detected_ball_color = None  # Valid: 'blue', 'green', 'yellow'
        self.ball_bbox = None   # (x, y, w, h) of detected ball
        self.ball_area = 0      # area of the detected ball bounding box
        self.image_width = None
        
        # Valid ball colors (red is considered invalid)
        self.valid_colors = ['blue', 'green', 'yellow']
        self.color_ranges = {
            'red':    ((0, 100, 100), (10, 255, 255)),
            'blue':   ((100, 150, 0), (140, 255, 255)),
            'green':  ((40, 70, 70), (80, 255, 255)),
            'yellow': ((20, 100, 100), (30, 255, 255))
        }
        
        # Duration settings (in seconds) for simulated pickup and drop actions
        self.pickup_duration = 3.0
        self.drop_duration = 3.0
        
        # LIDAR threshold for drop-off (in meters)
        self.dropoff_distance_threshold = 0.5
        
        # Threshold for considering the ball "close" based on bounding box area.
        # This value may require tuning.
        self.close_ball_area_threshold = 2000
        
        # Timer for state machine updates (every 0.5 sec)
        self.timer = self.create_timer(0.5, self.state_machine_callback)

    # ---------- IMAGE CALLBACK (Ball Detection) ----------
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
        
        # Save image width for later use
        self.image_width = cv_image.shape[1]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        ball_found = False
        
        # Reset previous detection data
        self.ball_bbox = None
        self.ball_area = 0
        self.detected_ball_color = None
        
        # Loop through valid colors only
        for color in self.valid_colors:
            lower, upper = self.color_ranges[color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:  # Filter small noise
                    (x, y, w, h) = cv2.boundingRect(cnt)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    self.ball_bbox = (x, y, w, h)
                    self.ball_area = area
                    self.detected_ball_color = color
                    ball_found = True
                    break
            if ball_found:
                break

        self.ball_detected = ball_found
        
        # Optionally display the image for debugging
        cv2.imshow("Ball Detection (Simulation)", cv_image)
        cv2.waitKey(1)

    # ---------- LIDAR CALLBACK ----------
    def lidar_callback(self, msg):
        if msg.ranges:
            self.last_lidar_distance = min(msg.ranges)
        else:
            self.last_lidar_distance = None

    # ---------- STATE MACHINE CALLBACK ----------
    def state_machine_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9  # in seconds
        self.get_logger().info(f"State: {self.state} | Elapsed: {elapsed:.1f} s")
        
        if self.state == "SEARCH":
            # Look for a valid ball
            if self.ball_detected:
                self.get_logger().info(f"Ball detected: {self.detected_ball_color}. Switching to APPROACH state.")
                self.state = "APPROACH"
                self.state_start_time = current_time
            else:
                # No ball detected; robot remains idle.
                self.publish_cmd_vel(0.0, 0.0)
                
        elif self.state == "APPROACH":
            if self.ball_detected and self.ball_bbox is not None and self.image_width is not None:
                x, y, w, h = self.ball_bbox
                ball_center_x = x + w / 2.0
                image_center_x = self.image_width / 2.0
                error = ball_center_x - image_center_x  # positive if ball is to the right
                
                # Proportional control: adjust angular speed based on error.
                # You may need to tune the gain value.
                angular_gain = 0.005
                angular_z = -angular_gain * error  # negative to turn towards the ball
                
                # Constant forward speed
                linear_x = 0.15
                
                # Publish movement command to approach the ball.
                self.publish_cmd_vel(linear_x, angular_z)
                self.get_logger().info(f"Approaching ball: error={error:.1f}, linear_x={linear_x}, angular_z={angular_z:.3f}")
                
                # If the ball appears large (close), transition to PICKUP.
                if self.ball_area > self.close_ball_area_threshold:
                    self.get_logger().info("Ball is close. Transitioning to PICKUP state.")
                    self.publish_cmd_vel(0.0, 0.0)
                    self.state = "PICKUP"
                    self.state_start_time = current_time
            else:
                # Lost the ball; return to SEARCH.
                self.get_logger().info("Ball lost during approach. Returning to SEARCH state.")
                self.publish_cmd_vel(0.0, 0.0)
                self.state = "SEARCH"
                self.state_start_time = current_time
                
        elif self.state == "PICKUP":
            # Simulate pickup for a fixed duration
            if elapsed < self.pickup_duration:
                self.get_logger().info("Picking up ball...")
                self.publish_cmd_vel(0.0, 0.0)
            else:
                self.get_logger().info("Pickup complete. Transitioning to MOVE_TO_DROP.")
                self.state = "MOVE_TO_DROP"
                self.state_start_time = current_time
                
        elif self.state == "MOVE_TO_DROP":
            # Move forward until LIDAR indicates the drop-off location is reached
            twist = Twist()
            twist.linear.x = 0.2  # Adjust forward speed as needed
            self.cmd_vel_pub.publish(twist)
            
            if hasattr(self, 'last_lidar_distance') and self.last_lidar_distance is not None:
                self.get_logger().info(f"LIDAR distance: {self.last_lidar_distance:.2f} m")
                if self.last_lidar_distance < self.dropoff_distance_threshold:
                    self.get_logger().info("Drop-off location reached. Stopping and transitioning to DROP.")
                    self.publish_cmd_vel(0.0, 0.0)
                    self.state = "DROP"
                    self.state_start_time = current_time
            else:
                self.get_logger().warn("No LIDAR data available during MOVE_TO_DROP.")
                
        elif self.state == "DROP":
            # Simulate drop action for a fixed duration
            if elapsed < self.drop_duration:
                self.get_logger().info("Dropping ball...")
                self.publish_cmd_vel(0.0, 0.0)
            else:
                self.get_logger().info("Drop complete. Resetting to SEARCH state.")
                self.ball_detected = False
                self.detected_ball_color = None
                self.ball_bbox = None
                self.ball_area = 0
                self.state = "SEARCH"
                self.state_start_time = current_time

    # ---------- Helper: Publish Velocity Command ----------
    def publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
    def destroy_node(self):
        cv2.destroyAllWindows()
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
