import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleGoalController(Node):
    def __init__(self):
        super().__init__('turtle_goal_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.tolerance = 0.1
        self.rate = self.create_rate(10)

    def pose_callback(self, msg):
        distance = math.sqrt((self.goal_x - msg.x)**2 + (self.goal_y - msg.y)**2)
        if distance > self.tolerance:
            # Proportional control for velocity and angular rate
            linear_vel = 0.5 * distance
            angular_vel = 4.0 * (math.atan2(self.goal_y - msg.y, self.goal_x - msg.x) - msg.theta)

            cmd_vel = Twist()
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            self.publisher_.publish(cmd_vel)
        else:
            self.get_logger().info("Goal reached!")

    def get_user_input(self):
        self.goal_x = float(input("Enter the goal x coordinate: "))
        self.goal_y = float(input("Enter the goal y coordinate: "))
        self.tolerance = float(input("Enter the tolerance value: "))

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleGoalController()
    controller.get_user_input()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
