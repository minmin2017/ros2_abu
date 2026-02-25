import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
       super().__init__('simple_publisher')
       self.pub = self.create_publisher(String, 'chatter', 10)
       self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROBOCON! {self.get_clock().now().nanoseconds}'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    rclpy.shutdown()