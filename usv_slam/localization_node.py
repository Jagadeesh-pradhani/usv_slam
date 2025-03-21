#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, Point
import tf_transformations

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher for estimated pose
        self.pose_pub = self.create_publisher(PoseStamped, '/estimated_pose', 10)

        # Internal storage
        self.map_data = None
        self.map_info = None
        self.last_estimated_pose = np.array([-2.0, -0.5, 0.0])  # [x, y, theta]
        self.scan_received = False

    def map_callback(self, msg: OccupancyGrid):
        # Save map info and convert data to numpy array for fast lookup
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.get_logger().info("Map received.")

    def scan_callback(self, msg: LaserScan):
        if self.map_data is None:
            self.get_logger().warn("No map received yet!")
            return

        # Convert LaserScan to point cloud in the laser frame
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment

        points = np.array(points)  # Shape: (N, 2)
        if points.size == 0:
            self.get_logger().warn("Empty scan!")
            return

        # Use a simple grid search around the last estimated pose
        best_score = -np.inf
        best_pose = self.last_estimated_pose.copy()

        # Define search window (in meters and radians)
        search_range = 0.2  # meters
        search_step = 0.05  # meters
        angle_range = math.radians(10)  # radians
        angle_step = math.radians(2)

        for dx in np.arange(-search_range, search_range + search_step, search_step):
            for dy in np.arange(-search_range, search_range + search_step, search_step):
                for dtheta in np.arange(-angle_range, angle_range + angle_step, angle_step):
                    candidate = self.last_estimated_pose + np.array([dx, dy, dtheta])
                    score = self.compute_score(points, candidate)
                    if score > best_score:
                        best_score = score
                        best_pose = candidate

        self.last_estimated_pose = best_pose
        self.publish_pose(best_pose)
        self.get_logger().info(f"Estimated pose: x={best_pose[0]:.2f}, y={best_pose[1]:.2f}, theta={math.degrees(best_pose[2]):.1f}°")

    def compute_score(self, points, pose):
        """
        Transform the scan points using the candidate pose and score the alignment with the map.
        A simple method: For each transformed point, sample the map occupancy value (assuming higher values are more likely obstacles)
        and sum these values. (You might want to invert this if free space is encoded as 0.)
        """
        # Unpack pose
        x, y, theta = pose

        # Create rotation matrix
        R = np.array([[math.cos(theta), -math.sin(theta)],
                      [math.sin(theta),  math.cos(theta)]])

        # Transform points from laser frame to map frame
        transformed_points = (R @ points.T).T + np.array([x, y])  # Shape: (N,2)

        score = 0.0
        # Map parameters
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        for pt in transformed_points:
            # Compute map cell indices
            map_x = int((pt[0] - origin_x) / resolution)
            map_y = int((pt[1] - origin_y) / resolution)

            # Check bounds
            if 0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height:
                # Here we assume that lower occupancy values indicate free space.
                # If your map uses 0 for free and 100 for occupied, you might want to use the negative occupancy value.
                occupancy = self.map_data[map_y, map_x]
                # For example, reward free space (0) and penalize obstacles (100)
                score += (100 - occupancy)
            else:
                # Out of bounds: penalize heavily
                score -= 100
        return score

    def publish_pose(self, pose):
        x, y, theta = pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position = Point(x=x, y=y, z=0.0)
        # Convert theta to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
