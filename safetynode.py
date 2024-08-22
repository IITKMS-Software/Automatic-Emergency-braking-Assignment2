#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safetynode')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        
        # TODO: create ROS subscribers and publishers.
        self.linear_velocity=0.0
        self.drive_pub=self.create_publisher(AckermannDriveStamped,"/drive",10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscribe to /scan and /ego_racecar/odom topics
        self.scan=self.create_subscription(LaserScan,"/scan",self.scan_callback,10)
        self.odom=self.create_subscription(Odometry,"/ego_racecar/odom",self.odom_callback,10)
        
    def brake(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.cmd_vel_pub.publish(msg)
        #self.get_logger().info(str(self.linear_velocity)+str("bra"))

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.linear_velocity = odom_msg.twist.twist.linear.x
        self.get_logger().info(str("Velocity is")+str(self.linear_velocity))

    def scan_callback(self, scan_msg):
        # TODO: calculate TTCo
        
       # self.get_logger().info(str(theta)+str("Angle"))
        arr_of_ran=np.array(scan_msg.ranges)
        r=min(arr_of_ran)
        for i in range(len(arr_of_ran)):
            if(arr_of_ran[i]==r):
                theta_index=i
                break
        theta=scan_msg.angle_min+i*scan_msg.angle_increment
        change_in_r=r/self.linear_velocity*math.cos(theta)
        change_in_r=(self.linear_velocity+1e-6)
        ittc=r/change_in_r
        self.get_logger().info(str("Value of iitc is")+str(ittc))

       
        

        if ittc < 1.00 and ittc>0.01:
            self.brake()
            self.get_logger().info(str("Brake"))

       


        
        # TODO: publish command to brake
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