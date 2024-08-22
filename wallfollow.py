
import rclpy
from rclpy.node import Node
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriberlaser=self.create_subscription(LaserScan,'/scan',self.scan_callback,10)

        # TODO: set PID gains
        self.kp = -6.0
        self.kd = 0.02
        self.ki = 10.0

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.walldist = 1.4
        self.l = 0.5
        self.info = LaserScan()
        self.time = 0.001

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        ranges=np.array(range_data)
        # print(type(ranges))
        index= int((angle-self.info.angle_min)/self.info.angle_increment)   
        # print(index)
        # self.get_logger().info(f"Angle_Min = {self.info.angle_min}, Index = {index}")
        if math.isfinite(ranges[index]):
            return ranges[index]
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        a = self.get_range(range_data,np.radians(90))
        b= self.get_range(range_data,np.radians(60))
        # c = self.get_range(range_data, np.radians(120))
        # d = self.get_range(range_data, np.radians(-70))
        #TODO:implement
        theta = 90-60
        alpha= np.arctan((a*np.cos(np.radians(theta))-b)/a*np.sin(np.radians(theta)))
        dt= b*np.cos(alpha)+self.l*np.sin(alpha)
        # self.get_logger().info(f"a = {a} b = {b}  alpha = {alpha} dt={dt}")
        return dist-dt

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = self.kp*error + self.kd*(self.error-self.prev_error)/self.time + self.ki*self.integral
        self.prev_error=self.error
        self.integral=self.integral+self.prev_error*self.time

        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed=self.get_velocity(angle)
        self.get_logger().info(f"Velocity - {self.get_velocity(angle)}, Angle = {np.degrees(angle)}")
        drive_msg.drive.steering_angle=angle
        # drive_msg.drive.steering_angle_velocity=angle/10
        # TODO: fill in drive message and publish
        self.publisher.publish(drive_msg)


    def get_velocity(self,angle):
        angle = np.degrees(angle)
        if angle < 10 and angle >= 0:
            return 1.5
        elif angle >= 10 and angle < 20:
            return 1.0
        else:
            return 0.5

        
    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.info=msg
        error = self.get_error(msg.ranges, self.walldist)
        self.get_logger().info(f"ERROR = {error}")# TODO: replace with error calculated by get_error()
        self.pid_control(error) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()