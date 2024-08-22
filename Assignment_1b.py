import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from turtlesim.srv import TeleportAbsolute
import random

class TurtlesimRandomControl(Node):
    def __init__(self):
        super().__init__('turtlesim_random_control')
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.center_sub = self.create_subscription(Pose, 'turtle1/pose', self.center_callback, 10)

    def center_callback(self, pose):
        self.get_logger().info(f"Turtle center position: ({pose.x}, {pose.y})")

    def get_random_velocities(self):
        linear_velocity = random.uniform(1, 3)
        angular_velocity = random.uniform(1, 3)
        return linear_velocity, angular_velocity

    def teleport_turtle_randomly(self):
        teleport_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for turtlesim teleport service...')

        request = TeleportAbsolute.Request()
        request.x = random.uniform(1, 10)
        request.y = random.uniform(1, 10)
        request.theta = 0.0

        future = teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def run_eight_shape_randomly(self):
        linear_velocity, angular_velocity = self.get_random_velocities()
        self.teleport_turtle_randomly()

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity

        duration = 5.0
        loop_rate = 10
        total_duration = 2 * duration

        start_time = self.get_clock().now().to_msg()
        end_time = self.get_clock().now().to_msg()

        while end_time.sec - start_time.sec < total_duration:
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info(f"Linear Velocity: {msg.linear.x}, Angular Velocity: {msg.angular.z}")
            self.get_logger().info("Moving in 8-shaped pattern...")
            self.spin_once()
            rclpy.sleep(1.0 / loop_rate)
            end_time = self.get_clock().now().to_msg()

        msg.linear.x = 0
        msg.angular.z = 0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("8-shaped pattern completed!")

    def spin_once(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimRandomControl()
    try:
        node.run_eight_shape_randomly()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
