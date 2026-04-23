import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver')
        
        # 1. สร้าง Publisher ส่งภาพไปที่ /camera/image_raw
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # 2. ตั้งค่า Timer ให้ทำงาน 30 FPS (1/30 = 0.033 วินาที)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # 3. เปิดกล้อง (0 คือกล้องเว็บแคมตัวแรก)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error('ไม่สามารถเปิดกล้องได้! กรุณาเช็คการเชื่อมต่อ')
        else:
            self.get_logger().info('เปิดกล้องสำเร็จ! เริ่มส่งภาพไปที่ /camera/image_raw')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # แปลงภาพ OpenCV เป็น ROS Image Message
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('ไม่สามารถอ่านเฟรมจากกล้องได้')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
