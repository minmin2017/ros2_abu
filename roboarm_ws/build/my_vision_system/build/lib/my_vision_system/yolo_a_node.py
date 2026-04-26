import rclpy
from rclpy.node import Node
import os
import time
import cv2
import torch
import glob
from ultralytics import YOLO

try:
    import serial as pyserial
    import serial.tools.list_ports
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

class YoloANode(Node):
    def __init__(self):
        super().__init__('yolo_a_node')

        # Parameters
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('camera_match', 'Jieli')
        self.declare_parameter('model_path', '/home/minmin/roboarm_ws/src/my_vision_system/models_upgrade/best.pt')
        self.declare_parameter('conf_threshold', 0.8)
        
        self.serial = None
        self._serial_buf = ''
        self.decided = False
        self.stage1_ready = False
        
        # YOLO Setup
        model_p = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'กำลังโหลดโมเดล YOLO: {model_p}')
        try:
            self.model = YOLO(model_p)
            self.device = 0 if torch.cuda.is_available() else 'cpu'
            if self.device == 0: self.model.to('cuda')
        except Exception as e:
            self.get_logger().error(f'โหลดโมเดลไม่สำเร็จ: {e}')
            os._exit(1)

        self._init_serial()
        
        if not self.serial:
            self.get_logger().error('❌ ไม่พบ Arduino! กรุณาเช็คสาย USB')
        else:
            self.get_logger().info('🚀 YoloA Node: รอสัญญาณ "state1" เพื่อเริ่มตรวจจับ "A" (Spear)')
            
        self.cap = None
        self._open_camera()

        self.timer = self.create_timer(0.01, self.timer_callback)

    def _init_serial(self):
        if not _SERIAL_OK: return
        
        port = None
        try:
            for p in serial.tools.list_ports.comports():
                desc = (p.description or '').lower()
                hwid = (p.hwid or '').lower()
                # ปรับการตรวจจับให้ครอบคลุมขึ้น
                if any(x in desc or x in hwid for x in ['arduino', 'mega', 'ch340', 'usb serial', 'usb-serial']):
                    port = p.device
                    break
        except Exception: pass
        
        if port:
            try:
                self.serial = pyserial.Serial(port, self.get_parameter('serial_baud').value, timeout=0)
                self.get_logger().info(f'✅ เชื่อมต่อ Serial: {port}')
            except Exception as e:
                self.get_logger().error(f'❌ เปิด Serial ไม่ได้: {e}')

    def _resolve_camera_path(self):
        match = self.get_parameter('camera_match').get_parameter_value().string_value
        candidates = sorted(glob.glob('/dev/v4l/by-id/*'))
        for link in candidates:
            if match.lower() in os.path.basename(link).lower() and 'index0' in link:
                return os.path.realpath(link)
        return None

    def _open_camera(self):
        path = self._resolve_camera_path()
        if path:
            self.cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.get_logger().info(f'✅ เปิดกล้องสำเร็จ: {path}')
                return True
        self.get_logger().error('❌ ไม่พบกล้อง Jieli!')
        return False

    def timer_callback(self):
        # 1. อ่าน Serial
        if self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                    self._serial_buf += data
                    if 'state1' in self._serial_buf.lower():
                        if not self.stage1_ready:
                            self.get_logger().info('✅ ได้รับ "state1" — เริ่มตรวจจับ YOLO...')
                            self.stage1_ready = True
            except Exception as e:
                self.get_logger().error(f'Serial Error: {e}')

        # 2. ถ้ายังไม่ได้รับ state1 ก็แค่อัปเดตจอ (ถ้ามีกล้อง)
        if not self.stage1_ready:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    cv2.putText(frame, "WAITING FOR STATE1...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                    cv2.imshow("YoloA Detection", frame)
                    cv2.waitKey(1)
            return

        # 3. ถ้าได้รับ state1 แล้ว และยังไม่ได้ตัดสินใจ
        if self.decided: return

        if self.cap is None or not self.cap.isOpened():
            self._open_camera()
            return

        ret, frame = self.cap.read()
        if not ret: return

        # YOLO Inference
        results = self.model.predict(
            frame, 
            conf=self.get_parameter('conf_threshold').value,
            device=self.device,
            verbose=False
        )

        found_spear = False
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id == 2: # Spear
                    found_spear = True
                    x1, y1, x2, y2 = box.xyxy[0]
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(frame, "SPEAR DETECTED!", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if found_spear:
            # ส่ง A ทันที
            if self.serial and self.serial.is_open:
                self.serial.write(b'A\n')
                self.get_logger().info('🎯 [SENT A] -> Arduino (Spear Detected)')
                self.decided = True
                
                # ปิด Node หลังจากส่งสำเร็จ
                self.get_logger().info('✅ ภารกิจเสร็จสิ้น. ปิด Node ใน 1 วินาที...')
                cv2.imshow("YoloA Detection", frame)
                cv2.waitKey(1000)
                os._exit(0)

        cv2.imshow("YoloA Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloANode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap: node.cap.release()
        if node.serial: node.serial.close()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
