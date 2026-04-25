import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess, re, cv2


def find_external_camera(logger=None):
    """เปิดกล้อง USB แยก โดยดูจาก v4l2-ctl แล้วทดสอบอ่านภาพจริง"""
    external = []
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, timeout=3)
        current = ''
        for line in result.stdout.splitlines():
            if not line.startswith('\t'):
                current = line
            else:
                m = re.search(r'/dev/video(\d+)', line)
                if m:
                    idx = int(m.group(1))
                    is_builtin = any(k in current for k in ('WebCam', 'webcam', 'Integrated', 'integrated'))
                    if not is_builtin:
                        external.append(idx)
    except Exception:
        external = list(range(2, 10))

    for idx in sorted(external):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        ret, _ = cap.read()   # ทดสอบอ่านจริง
        if ret:
            if logger:
                logger.info(f'กล้องแยกอยู่ที่ /dev/video{idx}')
            return cap
        cap.release()

    return None


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = find_external_camera(self.get_logger())
        if self.cap is None:
            self.get_logger().error('ไม่พบกล้องแยก!')
        else:
            self.get_logger().info('เริ่มส่งภาพ /camera/image_raw')

        self.create_timer(0.033, self.timer_callback)  # 30 fps

    def timer_callback(self):
        if self.cap is None:
            return
        ret, frame = self.cap.read()
        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        else:
            self.get_logger().warn('อ่านภาพไม่ได้')


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    if node.cap:
        node.cap.release()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
