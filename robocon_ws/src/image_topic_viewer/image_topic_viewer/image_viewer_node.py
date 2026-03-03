import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('image_viewer_node')

        self.declare_parameter('topic', '/camera')
        self.declare_parameter('window_name', 'ROS2 Image Viewer')

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Viewing images from topic: {self.topic}. '
            f'Close window or press Ctrl+C to stop.'
        )

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Failed to convert image message: {exc}')
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
