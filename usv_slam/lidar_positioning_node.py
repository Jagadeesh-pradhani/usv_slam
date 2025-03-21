#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPositioningNode(Node):
    def __init__(self):
        super().__init__('lidar_positioning_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Lidar Positioning Node started.")

    def scan_callback(self, msg: LaserScan):
        # Process LIDAR data to estimate drop-off location.
        # Here, we simply log the minimum distance as an example.
        if msg.ranges:
            min_distance = min(msg.ranges)
            self.get_logger().info(f"Min distance from LIDAR: {min_distance:.2f}")
        else:
            self.get_logger().warn("No LIDAR data received.")

def main(args=None):
    rclpy.init(args=args)
    node = LidarPositioningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down Lidar Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
