'''
THIS IS THE WORK OF MANVI BENGANI!!!! 
'''

import rclpy
from rclpy.node import Node
import math 
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
        self.get_logger().info(f"  Started")
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriberlaser=self.create_subscription(LaserScan,'/scan',self.scan_callback, 10)
        self.sub = self.create_subscription(Odometry,'/ego_racecar/odom',self.odom_callback, 10)
        self.speed = 0.0
        self.threshold = 0.75
        self.brake_exec = False
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        self.get_logger().info(f"Current Speed {round(self.speed, 2)}")

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ttc = np.array([])
        ranges = scan_msg.ranges
        angle = scan_msg.angle_min
        diff = scan_msg.angle_increment
        for index, i in enumerate(ranges):
            if math.isnan(i) or math.isinf(i):
                pass
            else:
                a = math.cos(angle+index*diff)
                rdot=self.speed*a
                rdot=max(rdot,0.0001)
                ttc =  np.append(ttc,i/rdot)

        min_range = np.argmin(ttc)

        if np.min(ttc)<self.threshold:
            brake=AckermannDriveStamped()
            stop=AckermannDrive()
            brake.drive = stop
            stop.speed = 0.0
            # self.timer = self.create_timer(self.timer_period, self.publish_commands)
            self.get_logger().info(f"BRAKE INITIALIZED")
            self.brake_exec = True
            self.publisher.publish(brake)
        if not self.brake_exec:
            brake=AckermannDriveStamped()
            stop=AckermannDrive()
            brake.drive = stop
            stop.speed=1.0
            stop.steering_angle = -0.1
            stop.steering_angle_velocity = 0.01
            self.publisher.publish(brake)
        # if self.speed >= 0.0 and self.speed <= 0.001:
        #     self.brake_exec = False

        
        self.get_logger().info(f"minrange = {ranges[min_range]}, ttc min={round(np.min(ttc), 4)}")
        # self.get_logger().info(f"lowest={scan_msg.range_min} highest={scan_msg.range_max} theta={scan_msg.angle_min} diff={ scan_msg.angle_increment}")
        # TODO: publish command to brake
        

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()