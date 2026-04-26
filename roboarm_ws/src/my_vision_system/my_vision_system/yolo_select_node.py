import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import os
import glob
import time
import logging
import subprocess
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
        self.declare_parameter('cap_fps', 60) # เพิ่มเป็น 60 FPS
        self.declare_parameter('conf_threshold', 0.8)
        self.declare_parameter('stable_frames', 20)
        self.declare_parameter('max_slots', 6)
        self.declare_parameter('priority_order', [0, 1, 2])  # Paper, Rock, Spear
        self.declare_parameter('grayscale', True)
        self.declare_parameter('imgsz', 640)
        # Physical layout: number of evenly-spaced slots across the frame.
        # Used to map detection x-center → absolute slot index (handles missing middle).
        self.declare_parameter('expected_slots', 3)

        # Serial Parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)

        # USB hard-reset (requires sudo). Pass via -p sudo_password:=... or env SUDO_PASS
        self.declare_parameter('sudo_password', '')
        self.declare_parameter('usb_reset_on_fail', True)
        self.declare_parameter('usb_reset_cooldown', 10.0)

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
        self.cam_path = None
        self.serial = None
        self._serial_buf = ''
        self._last_usb_reset = 0.0
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

    def _send_slot(self, slot, cls_id, layout):
        if self.serial is None or not self.serial.is_open: return
        
        # ปรับเป็น Class-based Mapping: Paper=C, Rock=B, Spear=A
        class_to_cmd = {0: 'C', 1: 'B', 2: 'A'}
        cmd = class_to_cmd.get(cls_id, 'A')
        
        # [SPECIAL RULE] If ONLY spears are detected, always send 'A'
        all_spears = all(d[1] == 2 for d in layout)
        
        if all_spears and cls_id == 2:
            cmd = 'A'
            self.get_logger().info(f'🎯 [SPEAR ONLY] -> Arduino: Character "{cmd}" (Override for Spear-only layout)')
        else:
            class_name = self.class_names.get(cls_id, f'Class {cls_id}')
            self.get_logger().info(f'🚀 [SENDING] -> Arduino: Class {class_name} mapped to Character "{cmd}"')
            
        try:
            self.serial.write(f'{cmd}\n'.encode())
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

    def _get_sudo_password(self):
        pwd = self.get_parameter('sudo_password').get_parameter_value().string_value
        if not pwd:
            pwd = os.environ.get('SUDO_PASS', '')
        return pwd

    def _find_camera_usb_sysfs(self):
        """หา USB sysfs path ของกล้องจาก product name match"""
        match = self.get_parameter('camera_match').get_parameter_value().string_value.lower()
        skip_csv = self.get_parameter('camera_skip').get_parameter_value().string_value
        skip_list = [s.strip().lower() for s in skip_csv.split(',') if s.strip()]
        for dev in glob.glob('/sys/bus/usb/devices/*'):
            prod_file = os.path.join(dev, 'product')
            if not os.path.exists(prod_file):
                continue
            try:
                with open(prod_file) as f:
                    product = f.read().strip().lower()
            except Exception:
                continue
            if match and match not in product:
                continue
            if any(s in product for s in skip_list):
                continue
            return dev
        return None

    def _can_reset_now(self):
        cd = self.get_parameter('usb_reset_cooldown').value
        return (time.time() - self._last_usb_reset) > cd

    def _reset_usb_camera(self):
        """Hard-reset กล้อง USB ผ่าน sysfs authorized (ต้อง sudo)"""
        if not self.get_parameter('usb_reset_on_fail').value:
            return False
        if not self._can_reset_now():
            return False
        sysfs = self._find_camera_usb_sysfs()
        if sysfs is None:
            self.get_logger().warning('USB reset: ไม่เจอ sysfs ของกล้อง — ข้าม', throttle_duration_sec=10.0)
            return False
        pwd = self._get_sudo_password()
        if not pwd:
            self.get_logger().warning('USB reset: ไม่มี sudo password — ตั้ง -p sudo_password:=... หรือ export SUDO_PASS', throttle_duration_sec=10.0)
            return False

        self.get_logger().info(f'🔄 USB reset: {sysfs}')
        if self.cap is not None:
            try: self.cap.release()
            except Exception: pass
            self.cap = None

        try:
            cmd = ['sudo', '-S', 'sh', '-c',
                   f'echo 0 > {sysfs}/authorized && sleep 0.5 && echo 1 > {sysfs}/authorized']
            r = subprocess.run(cmd, input=pwd + '\n', text=True,
                               capture_output=True, timeout=8)
            self._last_usb_reset = time.time()
            if r.returncode != 0:
                self.get_logger().warning(f'USB reset failed: {r.stderr.strip()}')
                return False
            self.get_logger().info('✅ USB reset เสร็จ — รอ udev re-enumerate')
            time.sleep(1.5)
            return True
        except Exception as e:
            self._last_usb_reset = time.time()
            self.get_logger().warning(f'USB reset exception: {e}')
            return False

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

    def _try_open(self, path):
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.get_parameter('cap_width').value)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('cap_height').value)
        cap.set(cv2.CAP_PROP_FPS,          self.get_parameter('cap_fps').value)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _open_camera(self):
        if self.cap is not None:
            try: self.cap.release()
            except Exception: pass
            self.cap = None

        path = self._resolve_camera_path()
        if path is not None:
            self.get_logger().info(f'กำลังพยายามเปิดกล้องที่: {path}')
            cap = self._try_open(path)
            if cap is not None:
                self.cap = cap
                self.cam_path = path
                self.get_logger().info('✅ เปิดกล้องสำเร็จ!')
                return True

        # Fallback: USB reset → re-enumerate → retry
        if self._reset_usb_camera():
            path = self._resolve_camera_path()
            if path is not None:
                cap = self._try_open(path)
                if cap is not None:
                    self.cap = cap
                    self.cam_path = path
                    self.get_logger().info('✅ เปิดกล้องสำเร็จหลัง USB reset!')
                    return True
        return False

    def _assign_slots(self, dets_sorted, max_slots):
        # dets_sorted: list[(x_center, cls_id)] ascending by x_center.
        # Map each detection to an absolute slot by dividing the frame width
        # into `expected_slots` equal columns. Handles missing-middle layouts
        # (paper+spear → slot 1+3) without needing to see the full set first.
        if not dets_sorted:
            return []
        expected = int(self.get_parameter('expected_slots').value)
        cw = float(self.get_parameter('cap_width').value)
        if expected <= 0 or cw <= 0:
            return [(i + 1, d[1]) for i, d in enumerate(dets_sorted)]
        slot_width = cw / expected
        result = []
        for d in dets_sorted:
            cx = d[0]
            slot = int(cx / slot_width) + 1
            slot = max(1, min(slot, expected))
            result.append((slot, d[1]))
        return result

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
        # Always poll serial (sets stage1_ready when "state1" line arrives)
        self._poll_serial()

        # Camera always-on (auto USB reset on failure, see _open_camera)
        if self.cap is None or not self.cap.isOpened():
            if not self._open_camera():
                self.get_logger().error('❌ เปิดกล้องไม่ได้!', throttle_duration_sec=3.0)
                self.frame_count += 1
                return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame — release & retry', throttle_duration_sec=3.0)
            try: self.cap.release()
            except Exception: pass
            self.cap = None
            return

        if self.get_parameter('grayscale').value:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        current_time = self.get_clock().now()
        diff = (current_time - self.prev_time).nanoseconds / 1e9
        fps = 1.0 / diff if diff > 0 else 0.0
        self.prev_time = current_time
        self.frame_count += 1

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # YOLO inference always-on
        conf_t = self.get_parameter('conf_threshold').value
        img_size = self.get_parameter('imgsz').value
        results = self.model.predict(
            source=frame,
            conf=conf_t,
            device=self.device,
            imgsz=img_size,
            iou=0.45,
            agnostic_nms=True, # ป้องกันการสลับ Class ไปมาในจุดเดียว
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

        if len(detections_data) > 0:
            msg = Float32MultiArray()
            msg.data = detections_data
            self.publisher_.publish(msg)

        # Decision/send: ONLY after Arduino + state1
        arduino_ok = self.serial is not None and self.serial.is_open
        ready_to_send = arduino_ok and self.stage1_ready

        if not arduino_ok:
            cv2.putText(frame, "WAITING: Arduino...", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        elif not self.stage1_ready:
            cv2.putText(frame, "WAITING: state1...", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        else:
            max_slots = self.get_parameter('max_slots').value
            sorted_dets = sorted(dets, key=lambda d: d[0])
            simple = [(d[0], d[2]) for d in sorted_dets]
            layout = self._assign_slots(simple, max_slots)
            sig = tuple(layout)
            
            if layout:
                self.missed_frames = 0
                self.history.append(sig)
            else:
                self.missed_frames = getattr(self, 'missed_frames', 0) + 1
                if self.missed_frames > 10:
                    self.history.clear()
                    self.decided = False

            stable_frames = self.get_parameter('stable_frames').value
            if len(self.history) >= 5:
                counts = {}
                for s in self.history:
                    counts[s] = counts.get(s, 0) + 1
                
                most_common_sig = max(counts, key=counts.get)
                frequency = counts[most_common_sig]

                if not self.decided:
                    if frequency >= (len(self.history) * 0.75) and len(self.history) >= 10:
                        priority = list(self.get_parameter('priority_order').value)
                        layout_list = list(most_common_sig)
                        slot, cls_id = self._decide(layout_list, priority)
                        if slot > 0:
                            self.decided = True
                            self.selected_slot = slot
                            self.selected_class = cls_id
                            self.final_layout = layout_list
                            self._send_slot(slot, cls_id, layout_list)
                    else:
                        pct = int(frequency/len(self.history)*100)
                        cv2.putText(frame, f"COLLECTING: {len(self.history)} ({pct}%)",
                                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                else:
                    cv2.putText(frame, f"DECIDED: slot {self.selected_slot}",
                                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
