import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import glob
import os
import torch
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. สร้างท่อส่งข้อมูล
        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)
        
        # 2. โหลดโมเดล AI + ส่งขึ้น GPU
        self.get_logger().info('กำลังโหลดโมเดล YOLO...')
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_directory = get_package_share_directory('my_vision_system')
            model_path = os.path.join(package_share_directory, 'models', 'best.pt')
            self.get_logger().info(f'Model path: {model_path}')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            self.model = YOLO('best.pt')

        self.device = 0 if torch.cuda.is_available() else 'cpu'
        if self.device == 0:
            self.model.to('cuda')
            # warmup
            self.model.predict(torch.zeros(1, 3, 640, 640, device='cuda'), verbose=False)
            self.get_logger().info(f'YOLO ใช้ GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.get_logger().warning('ไม่พบ CUDA — ใช้ CPU')

        # 3. ตั้งค่าพารามิเตอร์
        self.declare_parameter('camera_match', 'Jieli')
        self.declare_parameter('camera_path', '')
        self.declare_parameter('cap_width', 640)
        self.declare_parameter('cap_height', 480)
        self.declare_parameter('cap_fps', 60)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('grayscale', True)
        
        self.cap = None
        self.cam_path = None
        self._open_camera()

        # 4. Timer 60 Hz
        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)
        
        self.class_names = {0: 'PAPER', 1: 'ROCK', 2: 'SPEAR'}
        
        # ตัวแปรควบคุม
        self.prev_time = self.get_clock().now()
        self.frame_count = 0
        self.show_every_n = 5 # อัปเดต Feedback ทุก 5 เฟรม
        
        self.get_logger().info('YOLO Node พร้อมทำงาน! (Optimized Mode)')

    def _resolve_camera_path(self):
        explicit = self.get_parameter('camera_path').get_parameter_value().string_value
        if explicit and os.path.exists(explicit):
            return explicit
        match = self.get_parameter('camera_match').get_parameter_value().string_value
        candidates = sorted(glob.glob('/dev/v4l/by-id/*'))
        for link in candidates:
            if match.lower() in os.path.basename(link).lower() and 'index0' in link:
                return link
        return None

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
        path = self._resolve_camera_path()
        if path is None:
            return False
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.get_parameter('cap_width').value)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('cap_height').value)
        cap.set(cv2.CAP_PROP_FPS,          self.get_parameter('cap_fps').value)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        self.cam_path = path
        return True

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return
        ret, frame = self.cap.read()
        if not ret:
            self._open_camera()
            return
        
        if self.get_parameter('grayscale').value:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # คำนวณ FPS
        current_time = self.get_clock().now()
        diff = (current_time - self.prev_time).nanoseconds / 1e9
        fps = 1.0 / diff if diff > 0 else 0.0
        self.prev_time = current_time
        self.frame_count += 1

        # วาด FPS ลงบนเฟรมเสมอ (เพื่อให้ติดไปกับ debug image และ imshow)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Inference (ใช้ FP16)
        results = self.model.predict(
            frame,
            device=self.device,
            imgsz=self.get_parameter('imgsz').value,
            conf=self.get_parameter('conf_thresh').value,
            half=True if self.device == 0 else False,
            verbose=False,
        )
        
        detections_data = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)
                detections_data.extend([center_x, center_y, float(cls_id)])
                
                # วาดกรอบเฉพาะเฟรมที่จะโชว์
                if self.frame_count % self.show_every_n == 0:
                    class_name = self.class_names.get(cls_id, 'UNKNOWN')
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)
                    cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # ส่งข้อมูล
        if len(detections_data) > 0:
            msg = Float32MultiArray()
            msg.data = detections_data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Detected {len(detections_data)//3} objects | FPS: {fps:.1f}')
            if self.frame_count % self.show_every_n == 0:
                cv2.imwrite('/home/minmin/roboarm_ws/yolo_detection_debug.png', frame)
        else:
            self.get_logger().info(f'Searching... | FPS: {fps:.1f}', throttle_duration_sec=1.0)

        # อัปเดตจอ
        if self.frame_count % self.show_every_n == 0:
            cv2.imwrite('/home/minmin/roboarm_ws/camera_test.png', frame)
            cv2.imshow('Robot Vision', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
