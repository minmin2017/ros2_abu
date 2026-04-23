import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import os

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 1. Load Custom Weight (robocon_yolov11s.pt)
        # พยายามหาไฟล์ในโฟลเดอร์ปัจจุบันก่อน
        model_path = 'robocon_yolov11s.pt'
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'ไม่พบไฟล์ Weight: {model_path} กรุณาเอาไฟล์ไปวางไว้ที่ ~/my_camera_project')
            # fallback ไปใช้ตัวปกติก่อนเพื่อไม่ให้พัง
            model_path = 'yolo11n.pt'
        
        self.model = YOLO(model_path) 
        
        # Force GPU usage
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info(f'YOLOv11 กำลังใช้ Weight: {model_path} บน GPU (CUDA)')
        else:
            self.get_logger().warning('GPU not found, YOLOv11 is using CPU')

        # 2. Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 3. Publisher
        self.publisher_ = self.create_publisher(Image, '/yolo/debug_image', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info('YOLO Detector Node พร้อมทำงาน!')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Inference
            results = self.model(cv_image, verbose=False)

            # Draw results
            annotated_frame = results[0].plot()

            # Publish
            msg_out = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_.publish(msg_out)

            # Show window
            cv2.imshow("Robocon YOLOv11 Detection", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'YOLO Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
