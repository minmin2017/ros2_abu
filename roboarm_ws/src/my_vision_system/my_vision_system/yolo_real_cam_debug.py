import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
import subprocess
import re
import math
import os
import time
from ament_index_python.packages import get_package_share_directory

def find_external_camera():
    external = []
    try:
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True, timeout=2)
        current = ''
        for line in result.stdout.splitlines():
            if not line.startswith('\t'):
                current = line
            else:
                m = re.search(r'/dev/video(\d+)', line)
                if m:
                    idx = int(m.group(1))
                    is_builtin = any(k in current for k in ('WebCam', 'webcam', 'Integrated', 'integrated'))
                    if not is_builtin: external.append(idx)
    except:
        external = [0, 1, 2]

    for idx in sorted(external):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret: 
                return cap, idx
            cap.release()
    return None, -1

class RealCamDockingDebug(Node):
    def __init__(self):
        super().__init__('yolo_real_cam_debug')
        
        # Load Model
        package_share = get_package_share_directory('my_vision_system')
        model_path = os.path.join(package_share, 'models_upgrade', 'best.pt')
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)
        
        # Camera State
        self.cap = None
        self.cam_idx = -1
        self.last_reconnect_time = 0
        self.reconnect_interval = 2.0 # Try reconnect every 2 seconds
        
        # Timer for processing
        self.timer = self.create_timer(0.033, self.process_frame)
        self.target_class = 2 # Spear
        
        self.get_logger().info('Debug Node Started with Auto-Reconnect. Press Q to exit.')

    def try_reconnect(self):
        now = time.time()
        if now - self.last_reconnect_time < self.reconnect_interval:
            return

        self.last_reconnect_time = now
        self.get_logger().info('Searching for external camera...')
        cap, idx = find_external_camera()
        
        if cap is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap = cap
            self.cam_idx = idx
            self.get_logger().info(f'Successfully connected to /dev/video{idx}')
        else:
            self.get_logger().warn('External camera not found, retrying soon...')

    def process_frame(self):
        # 1. Check/Handle Connection
        if self.cap is None:
            self.try_reconnect()
            # Show "Searching" screen
            waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(waiting_img, "SEARCHING FOR CAMERA...", (150, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            cv2.imshow("YOLO Real-Cam Docking Debug", waiting_img)
            cv2.waitKey(1)
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error(f'Lost connection to /dev/video{self.cam_idx}')
            self.cap.release()
            self.cap = None
            return

        # 2. Grayscale Processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        processed_frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # 3. YOLO Inference
        results = self.model(processed_frame, conf=0.5, verbose=False)
        
        h, w = processed_frame.shape[:2]
        center_x = w // 2
        best_det = None

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                if cls == self.target_class:
                    best_det = box.xyxy[0].cpu().numpy()
                    break

        # 4. Drawing & Logic
        display_img = processed_frame.copy()
        cv2.line(display_img, (center_x, 0), (center_x, h), (255, 0, 0), 1) # Center Line
        cv2.putText(display_img, f"Source: /dev/video{self.cam_idx}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        if best_det is not None:
            x1, y1, x2, y2 = map(int, best_det)
            obj_cx = (x1 + x2) // 2
            obj_cy = (y1 + y2) // 2
            
            cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(display_img, (obj_cx, obj_cy), 5, (0, 255, 0), -1)
            
            error_x = obj_cx - center_x
            if abs(error_x) > 20:
                dir_text = "SLIDE RIGHT" if error_x > 0 else "SLIDE LEFT"
                color = (0, 0, 255)
                cv2.arrowedLine(display_img, (center_x, h//2), (obj_cx, h//2), color, 3)
                cv2.putText(display_img, f"{dir_text} ({error_x}px)", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            else:
                cv2.putText(display_img, "ALIGNED", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLO Real-Cam Docking Debug", display_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            if self.cap: self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = RealCamDockingDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap: node.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
