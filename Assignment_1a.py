import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlesimControl(Node):
    def __init__(self):
        super().__init__('turtlesim_control')
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def get_user_input(self):
        linear_velocity = input("Enter linear velocity (2-6): ")
        linear_velocity = max(2, min(6, float(linear_velocity)))

        angular_velocity = input("Enter angular velocity (1-3): ")
        angular_velocity = max(1, min(3, float(angular_velocity)))

        return linear_velocity, angular_velocity

    def run_eight_shape(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = float(linear_velocity)  # Convert to float
        msg.angular.z = float(angular_velocity)  # Convert to float

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
    node = TurtlesimControl()
    try:
        linear_velocity, angular_velocity = node.get_user_input()
        node.run_eight_shape(linear_velocity, angular_velocity)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
