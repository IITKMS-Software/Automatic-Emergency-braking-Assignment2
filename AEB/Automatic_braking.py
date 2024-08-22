#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.brake_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # TODO: create ROS subscribers and publishers.
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = 0.
        self.speed = odom_msg.twist.twist.linear.x 

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        min_scan_distance = 0.1  # Minimum acceptable scan distance (adjust as needed)
        speed_tolerance = 0.01  # A small value to avoid division by zero
        # Calculate TTC
        min_distance = min(scan_msg.ranges)
        if min_distance < min_scan_distance:
            # To avoid division by zero, set a minimum acceptable distance
            min_distance = min_scan_distance

        ttc = min_distance / (self.speed + speed_tolerance)

        # Publish brake command if TTC is below a threshold
        ttc_threshold = 1.0  
        if ttc < ttc_threshold:
            brake_command = AckermannDriveStamped()
            brake_command.header.stamp = self.get_clock().now().to_msg()
            brake_command.drive.speed = 0.0  # Set speed to 0 for braking
            self.brake_pub.publish(brake_command)

        
        # TODO: publish command to brake
        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()