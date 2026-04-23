import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # ดึง Type ข้อมูลรูปภาพมาใช้
from cv_bridge import CvBridge    # ตัวแปลงข้อมูล ROS เป็น OpenCV
import cv2                        # Library สำหรับจัดการรูปภาพ

class CameraSubNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # 1. สร้าง Subscriber
        # พารามิเตอร์: (ประเภทข้อความ, ชื่อ Topic, ฟังก์ชันที่ทำงานเมื่อภาพมา, คิวรอ)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # ต้องตรงกับชื่อ Topic ใน Gazebo
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info('Node กล้องเริ่มทำงานแล้ว...')

    def image_callback(self, msg):
        # 2. แปลงข้อมูลจาก ROS Image เป็น OpenCV format
        try:
            # msg คือข้อมูลดิบจาก ROS, "bgr8" คือ Format สีมาตรฐานของ OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 3. แสดงผลหน้าต่างภาพ (ใช้สำหรับ Debug)
            cv2.imshow("Camera View from Gazebo", cv_image)
            cv2.waitKey(1) # จำเป็นต้องมีเพื่อให้หน้าต่างค้างไว้

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()