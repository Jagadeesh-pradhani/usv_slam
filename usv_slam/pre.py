#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import numpy as np

def quaternion_to_euler(qx, qy, qz, qw):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    Returns (roll, pitch, yaw) in radians.
    """
    # Quaternion normalization (just in case)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber to IMU
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)

        # Current yaw from IMU
        self.current_yaw = 0.0

        # We’ll keep track of an initial heading so we can do relative turns
        self.initial_yaw_set = False
        self.initial_yaw = 0.0
        self.breadth = 150        # 1305
        self.length =  100         # 1120

        # State machine: each “step” is either a timed forward motion or a turn.
        # Steps: (type, value)
        #   type = 'forward' or 'turn'
        #   value = distance in cm (forward) or angle in degrees (turn)
        self.path_sequence = [
            ('forward', self.breadth/2),   # 1
            ('turn',   -90),    # 2 (negative => right turn)
            ('forward', self.length),  # 3
            ('turn',    90),    # 4 (left turn)
            ('forward', self.breadth/2),   # 5
            ('turn',    90),    # 6 (left turn)
            ('forward', self.length/2),   # 7
            ('turn',    90),    # 8 (left turn)
            ('forward', self.breadth),  # 9
            ('turn',   -90),    # 10 (right turn)
            ('forward', self.length/2),   # 11
            ('turn',   180),    # 12
            ('forward', self.length),  # 13
            ('stop',      0),   # 14
        ]

        # Convert distance to time-based approach:
        # Suppose we move at 0.2 m/s => 20 cm/s
        self.forward_speed_m_s = 0.2  # you can tune this
        self.turn_speed_rad_s = 0.3   # you can tune this

        # We’ll keep an index for the current step
        self.current_step = 0

        # For time-based motion
        self.motion_start_time = None
        self.motion_duration = 0.0

        # For turning, we track target yaw (relative to initial heading)
        self.target_yaw = None

        # Create a timer to run the state machine
        self.timer = self.create_timer(0.1, self.state_machine_callback)  # 10 Hz

    def imu_callback(self, msg: Imu):
        # Extract yaw from IMU
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        _, _, yaw = quaternion_to_euler(qx, qy, qz, qw)
        self.current_yaw = yaw

        # Set initial yaw if not set
        if not self.initial_yaw_set:
            self.initial_yaw = self.current_yaw
            self.initial_yaw_set = True

    def state_machine_callback(self):
        """
        Called periodically to check the current step of the path
        and command velocities.
        """
        if self.current_step >= len(self.path_sequence):
            # We are done with all steps
            self.stop_robot()
            return

        step_type, value = self.path_sequence[self.current_step]

        if step_type == 'stop':
            # Final step: stop the robot
            self.stop_robot()
            self.current_step += 1
            return

        if self.motion_start_time is None:
            # Initialize the motion
            self.motion_start_time = self.get_clock().now()
            if step_type == 'forward':
                # Convert cm to meters
                distance_m = value / 100.0
                # Time = distance / speed
                self.motion_duration = distance_m / self.forward_speed_m_s
            elif step_type == 'turn':
                # Convert degrees to radians
                angle_rad = math.radians(value)
                # We want to set a target yaw = initial_yaw + angle (relative)
                self.target_yaw = self.normalize_angle(self.initial_yaw + angle_rad)
                # Estimate time to turn
                self.motion_duration = abs(angle_rad) / self.turn_speed_rad_s

        elapsed_time = (self.get_clock().now() - self.motion_start_time).nanoseconds * 1e-9

        if step_type == 'forward':
            if elapsed_time < self.motion_duration:
                # Command forward velocity
                twist = Twist()
                twist.linear.x = self.forward_speed_m_s
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
            else:
                # Done moving forward, go to next step
                self.stop_robot()
                self.motion_start_time = None
                self.current_step += 1
                # Reset “initial_yaw” in case you want each turn to be relative
                # to the heading after forward motion:
                self.initial_yaw = self.current_yaw

        elif step_type == 'turn':
            # We turn in place until we achieve the desired angle or time
            current_yaw = self.current_yaw
            desired_yaw = self.target_yaw
            angle_diff = self.angle_diff(desired_yaw, current_yaw)

            # Determine turn direction (sign of angular.z)
            direction = 1.0 if angle_diff > 0.0 else -1.0
            # If within some small threshold, we consider the turn complete
            angle_threshold = math.radians(2.0)  # 2 degrees threshold

            if abs(angle_diff) > angle_threshold and elapsed_time < self.motion_duration:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = direction * self.turn_speed_rad_s
                self.cmd_vel_pub.publish(twist)
            else:
                # Done turning
                self.stop_robot()
                self.motion_start_time = None
                self.current_step += 1
                # Reset initial yaw for next step
                self.initial_yaw = self.current_yaw

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize an angle to the range [-pi, pi).
        """
        angle = math.fmod(angle + math.pi, 2.0 * math.pi)
        if angle < 0.0:
            angle += 2.0 * math.pi
        return angle - math.pi

    @staticmethod
    def angle_diff(a, b):
        """
        Compute the difference between two angles a and b (both in radians),
        in the range [-pi, pi).
        """
        diff = a - b
        diff = math.fmod(diff + math.pi, 2.0*math.pi)
        if diff < 0.0:
            diff += 2.0*math.pi
        return diff - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
