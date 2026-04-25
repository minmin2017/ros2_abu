import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import os
import glob
import time
import logging
from collections import deque

os.environ['YOLO_VERBOSE'] = 'False'
from ultralytics import YOLO
from ultralytics.utils import LOGGER as _ULTRA_LOGGER
_ULTRA_LOGGER.setLevel(logging.WARNING)

try:
    import serial as pyserial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False

class YoloSelectNode(Node):
    def __init__(self):
        super().__init__('yolo_select_node')

        # Parameters
        self.declare_parameter('model_path', '/home/minmin/roboarm_ws/src/my_vision_system/models_upgrade/best.pt')
        self.declare_parameter('camera_match', 'Jieli')
        self.declare_parameter('camera_skip', 'Azurewave,Integrated,IMC,Microsoft,Logitech_BRIO_Webcam_Front')
        self.declare_parameter('camera_path', '')
        self.declare_parameter('cap_width', 640)
        self.declare_parameter('cap_height', 480)
        self.declare_parameter('cap_fps', 30)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('stable_frames', 30)
        self.declare_parameter('max_slots', 6)
        self.declare_parameter('priority_order', [1, 2, 0])  # Paper, Rock, Spear
        self.declare_parameter('grayscale', True)

        # Serial Parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)

        # QoS for decision latching
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)
        self.position_pub = self.create_publisher(Int32, '/picking_position', latched_qos)
        self.layout_pub = self.create_publisher(String, '/picking_layout', latched_qos)

        # YOLO Setup
        model_p = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading model: {model_p}')
        self.model = YOLO(model_p)
        self.class_names = self.model.names
        self.device = 0 # force CPU for stability or 0 for GPU

        # Camera & Serial State
        self.cap = None
        self.serial = None
        self._serial_buf = ''
        self._init_serial()

        self.prev_time = self.get_clock().now()
        self.frame_count = 0
        self.show_every_n = 5

        sf = self.get_parameter('stable_frames').value
        self.history = deque(maxlen=sf)
        self.decided = False
        self.selected_slot = -1
        self.selected_class = -1
        self.final_layout = []
        self.stage1_ready = False

        self.get_logger().info('YOLO Select Node พร้อมทำงาน! (Stability + Decision Mode)')
        self.timer = self.create_timer(0.01, self.timer_callback)

    def _find_arduino_port(self):
        """หา port ของ Arduino Mega อัตโนมัติ"""
        ARDUINO_VID = 0x2341
        MEGA_PIDS = {0x0042, 0x0010, 0x0016}
        CH340_VID = 0x1A86
        CH340_PID = 0x7523

        if _SERIAL_OK:
            try:
                from serial.tools import list_ports
                all_ports = list(list_ports.comports())
                for p in all_ports:
                    if p.vid == ARDUINO_VID and p.pid in MEGA_PIDS:
                        return p.device
                for p in all_ports:
                    if p.vid == CH340_VID and p.pid == CH340_PID:
                        self.get_logger().info(f'พบ Arduino Clone (CH340) ที่ {p.device}')
                        return p.device
                for p in all_ports:
                    desc = (p.description or '').lower()
                    if 'arduino' in desc or 'mega' in desc:
                        return p.device
                usb_ports = [p.device for p in all_ports if 'USB' in (p.description or '') or 'ttyUSB' in p.device or 'ttyACM' in p.device]
                if len(usb_ports) == 1:
                    return usb_ports[0]
            except Exception as e:
                self.get_logger().error(f'Error searching ports: {e}')

        for link in sorted(glob.glob('/dev/serial/by-id/*')):
            name = os.path.basename(link).lower()
            if 'arduino' in name or 'mega' in name or 'ch340' in name or 'usb-serial' in name:
                return os.path.realpath(link)

        for dev in ['/dev/ttyUSB0', '/dev/ttyACM0']:
            if os.path.exists(dev): return dev

        hint = self.get_parameter('serial_port').get_parameter_value().string_value
        if hint and os.path.exists(hint): return hint
        return None

    def _init_serial(self):
        if not _SERIAL_OK: return
        port = self._find_arduino_port()
        if port is None: return
        baud = self.get_parameter('serial_baud').value
        try:
            if self.serial is not None: self.serial.close()
            self.serial = pyserial.Serial(port, baud, timeout=0)
            self.get_logger().info(f'Serial เชื่อมต่อ Arduino Mega ที่ {port} @ {baud}')
        except Exception as e:
            self.get_logger().warning(f'Serial เปิดไม่ได้ ({port}): {e}')
            self.serial = None

    def _send_slot(self, slot):
        if self.serial is None or not self.serial.is_open: return
        mapping = {1: 'C', 2: 'B', 3: 'A', 4: 'D', 5: 'E', 6: 'F'}
        cmd = mapping.get(slot, str(slot))
        try:
            self.serial.write(f'{cmd}\n'.encode())
            self.get_logger().info(f'🚀 [SENDING] -> Arduino: Slot {slot} mapped to Character "{cmd}"')
        except Exception as e:
            self.get_logger().warning(f'Serial write error: {e}')
            self.serial = None

    def _poll_serial(self):
        if not _SERIAL_OK: return
        if self.serial is None or not self.serial.is_open:
            if self.frame_count % 150 == 0: self._init_serial()
            return
        try:
            if self.serial.in_waiting > 0:
                self._serial_buf += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                while '\n' in self._serial_buf:
                    line, self._serial_buf = self._serial_buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.get_logger().info(f'Serial ← Arduino: "{line}"')
                        if 'state1' in line.lower():
                            self.stage1_ready = True
                            self.get_logger().info('✅ ได้รับ "state1" จาก Arduino — เริ่มทำงานได้!')
        except Exception:
            self.serial = None

    def _resolve_camera_path(self):
        explicit = self.get_parameter('camera_path').get_parameter_value().string_value
        if explicit and os.path.exists(explicit):
            return explicit

        match = self.get_parameter('camera_match').get_parameter_value().string_value
        skip_csv = self.get_parameter('camera_skip').get_parameter_value().string_value
        skip_list = [s.strip().lower() for s in skip_csv.split(',') if s.strip()]
        candidates = sorted(glob.glob('/dev/v4l/by-id/*'))
        index0 = [c for c in candidates if 'index0' in c]

        # 1) preferred match (e.g. Jieli)
        for link in index0:
            if match.lower() in os.path.basename(link).lower():
                self.get_logger().info(f"เลือกกล้อง (match='{match}'): {os.path.basename(link)}", throttle_duration_sec=5.0)
                return link

        # 2) fallback: any external cam that is NOT in skip list (built-in webcams)
        for link in index0:
            name = os.path.basename(link).lower()
            if not any(s in name for s in skip_list):
                self.get_logger().warning(f"ไม่เจอ '{match}' — ใช้กล้องสำรอง: {os.path.basename(link)}", throttle_duration_sec=5.0)
                return link

        self.get_logger().error(
            f"Cannot find camera matching '{match}' (skip={skip_list}). Available: {[os.path.basename(c) for c in index0]}",
            throttle_duration_sec=3.0,
        )
        return None

    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
        path = self._resolve_camera_path()
        if path is None:
            return False

        self.get_logger().info(f'กำลังพยายามเปิดกล้องที่: {path}')
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
        self.get_logger().info('✅ เปิดกล้องสำเร็จ!')
        return True

    def _assign_slots(self, dets, max_slots):
        # simple heuristic: divide width into max_slots
        if not dets: return []
        slots = []
        for cx, cls_id in dets:
            slot_idx = int((cx / 640.0) * max_slots) + 1
            slot_idx = max(1, min(slot_idx, max_slots))
            slots.append((slot_idx, cls_id))
        return slots

    def _decide(self, layout, priority):
        for p_cls in priority:
            cls_to_slots = {}
            for slot, cls_id in layout:
                if cls_id not in cls_to_slots: cls_to_slots[cls_id] = []
                cls_to_slots[cls_id].append(slot)
            if p_cls in cls_to_slots:
                return (min(cls_to_slots[p_cls]), p_cls)
        return (-1, -1)

    def timer_callback(self):
        # 1. Check Arduino
        self._poll_serial()
        if self.serial is None or not self.serial.is_open:
            self.frame_count += 1
            self.get_logger().info('รอเชื่อมต่อ Arduino Mega...', throttle_duration_sec=3.0)
            return

        # 2. Wait for "state1" from Arduino before running YOLO
        if not self.stage1_ready:
            self.frame_count += 1
            self.get_logger().info('รอสัญญาณ "state1" จาก Arduino...', throttle_duration_sec=3.0)
            return

        # 3. Check & Open Camera
        if self.cap is None or not self.cap.isOpened():
            if not self._open_camera():
                self.get_logger().error('❌ [CRITICAL] เปิดกล้องไม่ได้!', throttle_duration_sec=3.0)
                return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from camera', throttle_duration_sec=3.0)
            return

        if self.get_parameter('grayscale').value:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        current_time = self.get_clock().now()
        diff = (current_time - self.prev_time).nanoseconds / 1e9
        fps = 1.0 / diff if diff > 0 else 0.0
        self.prev_time = current_time
        self.frame_count += 1

        # YOLO Inference
        conf_t = self.get_parameter('conf_threshold').value
        results = self.model.predict(
            source=frame,
            conf=conf_t,
            device=self.device,
            imgsz=640,
            half=True if self.device == 0 else False,
            verbose=False,
        )

        detections_data = []
        dets = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                detections_data.extend([cx, cy, float(cls_id)])
                dets.append((cx, cy, cls_id, conf, float(x1), float(y1), float(x2), float(y2)))

                class_name = self.class_names.get(cls_id, 'UNKNOWN')
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)
                cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        max_slots = self.get_parameter('max_slots').value
        sorted_dets = sorted(dets, key=lambda d: d[0])
        simple = [(d[0], d[2]) for d in sorted_dets]
        layout = self._assign_slots(simple, max_slots)
        sig = tuple(layout)

        if not self.decided:
            self.history.append(sig)
            stable_frames = self.get_parameter('stable_frames').value
            stable = (len(self.history) == stable_frames and len(set(self.history)) == 1 and len(layout) > 0)

            if stable:
                priority = list(self.get_parameter('priority_order').value)
                slot, cls_id = self._decide(layout, priority)
                if slot > 0:
                    self.decided = True
                    self.selected_slot = slot
                    self.selected_class = cls_id
                    self.final_layout = layout
                    self._send_slot(slot)
            else:
                seen_unique = len(set(self.history))
                cv2.putText(frame, f"Stab: {len(self.history)}/{stable_frames} uniq={seen_unique}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(frame, f"DECIDED: slot {self.selected_slot}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if len(detections_data) > 0:
            msg = Float32MultiArray()
            msg.data = detections_data
            self.publisher_.publish(msg)

        cv2.imshow('Robot Vision (Select)', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        if self.cap: self.cap.release()
        if self.serial: self.serial.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloSelectNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
