import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('camera_publisher_node')

        self.declare_parameter('topic', '/camera')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('fps', 120.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(int(camera_index))

        if not self.capture.isOpened():
            raise RuntimeError(
                f'Unable to open camera index {camera_index}. '
                'Check camera connection and permissions.'
            )

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.capture.set(cv2.CAP_PROP_FPS, float(fps))

        timer_period = 1.0 / max(fps, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_frame)
        self.get_logger().info(
            f'Publishing raw camera image to {topic} at {fps:.1f} FPS '
            f'({width}x{height})'
        )

    def publish_frame(self) -> None:
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning('Failed to read frame from camera')
            return

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.frame_id
        self.publisher_.publish(image_msg)

    def destroy_node(self) -> bool:
        if hasattr(self, 'capture') and self.capture.isOpened():
            self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = CameraPublisherNode()
    except RuntimeError as exc:
        temp_node = Node('camera_publisher_error_logger')
        temp_node.get_logger().error(str(exc))
        temp_node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
