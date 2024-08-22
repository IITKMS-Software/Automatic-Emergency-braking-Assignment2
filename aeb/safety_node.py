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
        self.get_logger().info("Safety Node Started")
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        self.threshold = 1.0
        # TODO: create ROS subscribers and publishers.

        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 100)
        self.speed_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 100)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        # self.get_logger().info(f"odom_callback: {odom_msg}")
        self.speed = odom_msg.twist.twist.linear.x
        # self.get_logger().info(f"speed: {self.speed}")

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges

        # TODO: calculate TTC
        # self.get_logger().info(f"scan_callback: {scan_msg.angle_min} {scan_msg.angle_max} {scan_msg.angle_increment} {scan_msg.range_min} {scan_msg.range_max}")

        ttc = np.inf

        for i in range(len(ranges)):
            theta = scan_msg.angle_min + i * scan_msg.angle_increment
            rdot = self.speed * np.cos(theta)
            r = ranges[i]
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue
            if rdot <= 10**-4:
                continue

            ttc = min(ttc, r / rdot)

        # TODO: publish command to brake
        if ttc < self.threshold:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.get_logger().info(f"brake: {ttc}")
            self.publisher.publish(msg)

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