import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
import math

class TurtlesimSquare(Node):
    def __init__(self):
        super().__init__('turtlesim_square')
        self.cmd_vel_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def get_user_input(self):
        length = input("Enter the length of the side of the square (1-5): ")
        length = max(1, min(5, float(length)))
        return length

    def draw_square(self, length):
        set_pen_service = self.create_client(SetPen, 'turtle1/set_pen')
        while not set_pen_service.wait_for_service(timeout_sec=1.0):
            pass

        request = SetPen.Request()
        request.r = 255
        request.g = 0
        request.b = 0
        request.width = 3
        request.off = 0

        for _ in range(4):
            msg = Twist()  # Create a new Twist message
            msg.linear.x = length
            self.cmd_vel_pub.publish(msg)
            rclpy.sleep(length / 2.0)

            msg.linear.x = 0
            self.cmd_vel_pub.publish(msg)
            rclpy.sleep(1.0)

            msg.angular.z = math.pi / 2.0
            self.cmd_vel_pub.publish(msg)
            rclpy.sleep(1.0)

            msg.angular.z = 0

            if _ == 0:
                set_pen_future = set_pen_service.call_async(request)
                rclpy.spin_until_future_complete(self, set_pen_future)

    def spin_once(self):
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimSquare()
    try:
        length = node.get_user_input()
        node.draw_square(length)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
