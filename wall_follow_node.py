import rclpy
from rclpy.node import Node
import math as math
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
        #Publisher
        self.controller=self.create_publisher(AckermannDriveStamped,drive_topic,10)


        #Subscriber
        self.scan=self.create_subscription(LaserScan,"/scan",self.scan_callback,10)

        # TODO: set PID gains
        self.kp = -5.0
        self.kd = 0.02
        self.ki = 7.0

        # TODO: store history
        #self.integral = 
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        index=(angle-self.min_angle)//self.theta_increment
        return (range_data[int(index)])

        #TODO: implement


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
        d=self.b*math.cos(self.alpha)
        d_next=d+0.5*math.sin(self.alpha)
        error=dist-(d_next)
        return (error)
        #TODO:implement
        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        del_t=0.001
        derivative_error=(self.error-self.prev_error)/del_t
        angle=self.kp*error+self.kd*derivative_error

        if angle*57>0 and angle*57<10:
            velocity=1.5
        elif angle*57>10 and angle*57<20:
            velocity=1.0
        else:
            velocity=0.5
        self.prev_error=error
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        drive_msg.drive.speed=velocity
        self.get_logger().info(str(velocity))
        drive_msg.drive.steering_angle=angle
        self.controller.publish(drive_msg)



    def scan_callback(self, scan_msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message


        Returns:
            None
        """
        array_of_ranges=np.array(scan_msg.ranges)
        self.min_angle=scan_msg.angle_min
        self.max_angle=scan_msg.angle_max
        self.angle=self.max_angle-self.min_angle
        self.theta_increment=scan_msg.angle_increment
        self.a=self.get_range(array_of_ranges,math.radians(30))
        self.b=self.get_range(array_of_ranges,math.radians(90))
        self.alpha=math.atan((self.a*math.cos(math.radians(self.angle))-self.b)/self.a*math.sin(math.radians(self.angle)))


        self.error = self.get_error(array_of_ranges,0.8) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        
        self.pid_control(self.error, velocity) # TODO: actuate the car with PID


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