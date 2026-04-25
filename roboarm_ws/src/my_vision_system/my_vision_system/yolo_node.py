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
            import os
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
            # warmup เพื่อ JIT/cuDNN ให้ลื่นตั้งแต่เฟรมแรก
            self.model.predict(torch.zeros(1, 3, 640, 640, device='cuda'), verbose=False)
            self.get_logger().info(f'YOLO ใช้ GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.get_logger().warning('ไม่พบ CUDA — ใช้ CPU (ช้ากว่ามาก)')

        # 3. เปิดกล้อง external (จับด้วย stable symlink ใน /dev/v4l/by-id/ — index แบบตัวเลขเปลี่ยนทุกรอบ)
        self.declare_parameter('camera_match', 'Jieli')   # substring ใน by-id name (DV20 = Jieli)
        self.declare_parameter('camera_path', '')         # ถ้าระบุตรงๆ จะข้าม match
        self.declare_parameter('cap_width', 1280)         # ความละเอียดที่ขอจากกล้อง
        self.declare_parameter('cap_height', 720)
        self.declare_parameter('cap_fps', 30)
        self.declare_parameter('imgsz', 960)              # YOLO inference resolution (สูง = แม่นกว่า, ช้ากว่า)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('grayscale', True)         # แปลงเป็น B&W ก่อนเข้า YOLO
        self.cap = None
        self.cam_path = None
        self._open_camera()

        # 4. timer 30 Hz (GPU เร็วพอ); inference จะ skip ถ้ายังประมวลผลเฟรมก่อนหน้าไม่เสร็จ
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        
        self.class_names = {0: 'PAPER', 1: 'ROCK', 2: 'SPEAR'}
        
        self.get_logger().info('YOLO Node พร้อมทำงาน! กำลังค้นหาเป้าหมาย...')

    def _resolve_camera_path(self):
        explicit = self.get_parameter('camera_path').get_parameter_value().string_value
        if explicit:
            return explicit if os.path.exists(explicit) else None
        match = self.get_parameter('camera_match').get_parameter_value().string_value
        # prefer video-index0 (capture node) over index1 (metadata)
        candidates = sorted(glob.glob('/dev/v4l/by-id/*'))
        for link in candidates:
            if match.lower() in os.path.basename(link).lower() and 'index0' in link:
                return link
        for link in candidates:
            if match.lower() in os.path.basename(link).lower():
                return link
        return None

    def _open_camera(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        path = self._resolve_camera_path()
        if path is None:
            self.get_logger().warning('ยังไม่เจอกล้อง external ใน /dev/v4l/by-id/ — จะลองใหม่')
            return False
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warning(f'เปิดกล้องไม่ได้: {path}')
            return False
        # บังคับ MJPG (USB bandwidth ไหวที่ res สูง) + ขอ resolution/fps ตาม param
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.get_parameter('cap_width').value)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('cap_height').value)
        cap.set(cv2.CAP_PROP_FPS,          self.get_parameter('cap_fps').value)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # ลด latency, ดึงเฟรมล่าสุดเสมอ
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.cap = cap
        self.cam_path = path
        self.get_logger().info(
            f'เชื่อมต่อกล้อง: {path} -> {os.path.realpath(path)} '
            f'@ {actual_w}x{actual_h} {actual_fps:.0f}fps MJPG'
        )
        return True

    def timer_callback(self):
        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('อ่านเฟรมไม่ได้ — กำลัง reconnect...')
            self._open_camera()
            return
        
        # แปลงเป็น B&W ทั้งหมด (gray → BGR 3 channels เพื่อให้ตรงกับ input shape ของโมเดลและ overlay)
        if self.get_parameter('grayscale').value:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # บันทึกภาพทดสอบทุกเฟรมเพื่อเช็คกล้อง
        cv2.imwrite('/home/minmin/roboarm_ws/camera_test.png', frame)

        # ให้ YOLO วิเคราะห์ภาพ (บน GPU, imgsz สูงเพื่อความแม่นยำ)
        results = self.model.predict(
            frame,
            device=self.device,
            imgsz=self.get_parameter('imgsz').value,
            conf=self.get_parameter('conf_thresh').value,
            verbose=False,
        )
        
        detections_data = []
        
        # วนลูปหาของทุกชิ้นในภาพ
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                
                # คำนวณหาจุดกึ่งกลาง
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)
                
                # เก็บข้อมูลลง List: [X, Y, Class_ID] ต่อท้ายกันไป
                detections_data.extend([center_x, center_y, float(cls_id)])
                
                # --- ส่วนแสดงผลบนจอ (วาดทุกอันที่เจอ) ---
                class_name = self.class_names.get(cls_id, 'UNKNOWN')
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 0), -1)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # ถ้าเจออย่างน้อย 1 ชิ้น
        if len(detections_data) > 0:
            # สร้างแพ็กเกจข้อความ
            msg = Float32MultiArray()
            msg.data = detections_data
            
            # ส่งข้อมูลลงท่อให้ Picking Node
            self.publisher_.publish(msg)
            
            num_objects = len(detections_data) // 3
            self.get_logger().info(f'Detected {num_objects} objects. Data sent.')
            
            # บันทึกภาพลงไฟล์เพื่อให้ผู้ใช้ดูได้
            cv2.imwrite('/home/minmin/roboarm_ws/yolo_detection_debug.png', frame)

        # โชว์ภาพวิดีโอ (ถ้า WSL ของคุณเด้งหน้าต่างไม่ได้ ให้ใส่เครื่องหมาย # ปิด 2 บรรทัดนี้ไว้)
        cv2.imshow('Robot Vision', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # ปิดกล้องอย่างปลอดภัยเมื่อปิดโปรแกรม
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