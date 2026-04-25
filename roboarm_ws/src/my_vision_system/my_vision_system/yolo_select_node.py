import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32, String
import cv2
import glob
import os
import torch
from collections import deque
from ultralytics import YOLO

try:
    import serial as pyserial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False


class YoloSelectNode(Node):
    """
    Clone of yolo_node + decision mode.

    Pipeline:
      1. YOLO inference each frame (same as yolo_node).
      2. Sort detections left-to-right by x-center, infer slot positions
         from the gaps between detections (a missing object widens a gap,
         so its slot is skipped).
      3. When the same layout signature repeats for `stable_frames`
         consecutive frames, enter DECIDED mode and pick a slot using
         `priority_order` (default SPEAR > ROCK > PAPER), preferring the
         leftmost slot if a class appears more than once.
      4. Publish the chosen 1-indexed slot on /picking_position (Int32)
         and the resolved layout "slot:CLASS,slot:CLASS,..." on
         /picking_layout (String). /detected_object is still published
         every frame for backwards compatibility.
    """

    def __init__(self):
        super().__init__('yolo_select_node')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)
        # Latched QoS so late subscribers still get the decision
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.position_pub = self.create_publisher(Int32, '/picking_position', latched_qos)
        self.layout_pub = self.create_publisher(String, '/picking_layout', latched_qos)

        self.get_logger().info('กำลังโหลดโมเดล YOLO...')
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_directory = get_package_share_directory('my_vision_system')
            model_path = os.path.join(package_share_directory, 'models_upgrade', 'best.pt')
            self.get_logger().info(f'Model path: {model_path}')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            self.model = YOLO('best.pt')

        self.device = 0 if torch.cuda.is_available() else 'cpu'
        if self.device == 0:
            self.model.to('cuda')
            self.model.predict(torch.zeros(1, 3, 640, 640, device='cuda'), verbose=False)
            self.get_logger().info(f'YOLO ใช้ GPU: {torch.cuda.get_device_name(0)}')
        else:
            self.get_logger().warning('ไม่พบ CUDA — ใช้ CPU')

        self.declare_parameter('camera_match', 'Jieli')
        self.declare_parameter('camera_path', '')
        self.declare_parameter('cap_width', 640)
        self.declare_parameter('cap_height', 480)
        self.declare_parameter('cap_fps', 30)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('grayscale', True)
        self.declare_parameter('stable_frames', 10)
        self.declare_parameter('max_slots', 6)
        # Priority list, highest first. Default: SPEAR(2) > ROCK(1) > PAPER(0)
        self.declare_parameter('priority_order', [2, 1, 0])
        # Serial to Arduino Mega (serial_port = hint/fallback ถ้า auto-detect ไม่เจอ)
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)

        self.serial = None
        self._serial_buf = ''
        self._init_serial()

        self.cap = None
        self.cam_path = None
        self._open_camera()

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

        self.class_names = {0: 'PAPER', 1: 'ROCK', 2: 'SPEAR'}

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

    def _find_arduino_port(self):
        """หา port ของ Arduino Mega อัตโนมัติ (VID/PID → by-id symlink → hint parameter)"""
        ARDUINO_VID = 0x2341
        MEGA_PIDS = {0x0042, 0x0010, 0x0016}  # Mega2560 / old bootloader / Mega ADK

        # 1. pyserial list_ports — แม่นที่สุด
        if _SERIAL_OK:
            try:
                from serial.tools import list_ports
                for p in list_ports.comports():
                    if p.vid == ARDUINO_VID and p.pid in MEGA_PIDS:
                        return p.device
                    desc = (p.description or '').lower()
                    if 'arduino' in desc or 'mega' in desc:
                        return p.device
            except Exception:
                pass

        # 2. /dev/serial/by-id/ symlinks
        for link in sorted(glob.glob('/dev/serial/by-id/*')):
            name = os.path.basename(link).lower()
            if 'arduino' in name or 'mega' in name:
                return os.path.realpath(link)

        # 3. hint parameter
        hint = self.get_parameter('serial_port').get_parameter_value().string_value
        if hint and os.path.exists(hint):
            return hint

        return None

    def _init_serial(self):
        if not _SERIAL_OK:
            self.get_logger().warning('pyserial ไม่ได้ติดตั้ง — ไม่มี Serial (pip install pyserial)')
            return
        port = self._find_arduino_port()
        if port is None:
            self.get_logger().warning('ไม่พบ Arduino Mega — จะลองใหม่อัตโนมัติทุก 5 วิ')
            return
        baud = self.get_parameter('serial_baud').value
        try:
            if self.serial is not None:
                try: self.serial.close()
                except Exception: pass
            self.serial = pyserial.Serial(port, baud, timeout=0)
            self.get_logger().info(f'Serial เชื่อมต่อ Arduino Mega ที่ {port} @ {baud}')
            # ถ้าตัดสินใจไปแล้วก่อน reconnect → ส่ง slot ซ้ำ
            if self.decided and self.selected_slot > 0:
                self._send_slot(self.selected_slot)
        except Exception as e:
            self.get_logger().warning(f'Serial เปิดไม่ได้ ({port}): {e}')
            self.serial = None

    def _send_slot(self, slot):
        if self.serial is None or not self.serial.is_open:
            return
        try:
            self.serial.write(f'{slot}\n'.encode())
            self.get_logger().info(f'Serial → Arduino: slot {slot}')
        except Exception as e:
            self.get_logger().warning(f'Serial write error: {e}')
            try: self.serial.close()
            except Exception: pass
            self.serial = None

    def _poll_serial(self):
        """อ่าน ACK จาก Arduino non-blocking + autoreconnect ทุก ~5 วิ"""
        if not _SERIAL_OK:
            return

        # Autoreconnect เมื่อ port หาย (~150 frames ≈ 5 วิ ที่ 30 fps)
        if self.serial is None or not self.serial.is_open:
            if self.frame_count % 150 == 0:
                self.get_logger().info('Serial: ลองเชื่อมต่อ Arduino ใหม่...')
                self._init_serial()
            return

        try:
            if self.serial.in_waiting > 0:
                self._serial_buf += self.serial.read(self.serial.in_waiting).decode(errors='ignore')
                while '\n' in self._serial_buf:
                    line, self._serial_buf = self._serial_buf.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.get_logger().info(f'Serial ← Arduino: "{line}"')
                        if 'stage1' in line.lower():
                            self.stage1_ready = True
                            self.get_logger().info('✅ ได้รับ "stage1" จาก Arduino — เริ่มทำงานได้!')
        except Exception as e:
            self.get_logger().warning(f'Serial read error: {e} — จะ reconnect')
            try: self.serial.close()
            except Exception: pass
            self.serial = None

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

    def _assign_slots(self, dets_sorted, max_slots):
        # dets_sorted: list[(x_center, cls_id)] ascending by x_center.
        # Use the smallest non-zero gap as the "1 slot" unit, then
        # round each gap to an integer step. Missing slots therefore
        # show up as an extra step.
        if not dets_sorted:
            return []
        if len(dets_sorted) == 1:
            return [(1, dets_sorted[0][1])]

        xs = [d[0] for d in dets_sorted]
        gaps = [xs[i+1] - xs[i] for i in range(len(xs)-1)]
        positive = [g for g in gaps if g > 0]
        unit = min(positive) if positive else 1.0

        slots = [1]
        for g in gaps:
            step = max(1, int(round(g / unit)))
            slots.append(slots[-1] + step)

        return [(min(slots[i], max_slots), dets_sorted[i][1]) for i in range(len(dets_sorted))]

    def _decide(self, layout, priority):
        # layout: list[(slot, cls_id)]; priority: list[int] highest-first
        cls_to_slots = {}
        for slot, cls_id in layout:
            cls_to_slots.setdefault(cls_id, []).append(slot)
        for cls_id in priority:
            if cls_id in cls_to_slots:
                return (min(cls_to_slots[cls_id]), cls_id)
        return (-1, -1)

    def timer_callback(self):
        # รอ Serial ก่อน — ยังไม่เชื่อมต่อ Arduino ไม่ทำอะไรทั้งนั้น
        self._poll_serial()
        if self.serial is None or not self.serial.is_open:
            self.frame_count += 1
            self.get_logger().info('รอเชื่อมต่อ Arduino Mega...', throttle_duration_sec=3.0)
            return

        if not self.stage1_ready:
            self.frame_count += 1
            self.get_logger().info('รอกสัญญาณ "stage1" จาก Arduino...', throttle_duration_sec=3.0)
            return

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

        current_time = self.get_clock().now()
        diff = (current_time - self.prev_time).nanoseconds / 1e9
        fps = 1.0 / diff if diff > 0 else 0.0
        self.prev_time = current_time
        self.frame_count += 1

        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        results = self.model.predict(
            frame,
            device=self.device,
            imgsz=self.get_parameter('imgsz').value,
            conf=self.get_parameter('conf_thresh').value,
            half=True if self.device == 0 else False,
            verbose=False,
        )

        detections_data = []
        dets = []  # (cx, cy, cls_id, conf, x1, y1, x2, y2)
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]
                cx = float((x1 + x2) / 2.0)
                cy = float((y1 + y2) / 2.0)
                detections_data.extend([cx, cy, float(cls_id)])
                dets.append((cx, cy, cls_id, conf, float(x1), float(y1), float(x2), float(y2)))

                if self.frame_count % self.show_every_n == 0:
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
            stable = (
                len(self.history) == stable_frames
                and len(set(self.history)) == 1
                and len(layout) > 0
            )
            if stable:
                priority = list(self.get_parameter('priority_order').value)
                slot, cls_id = self._decide(layout, priority)
                if slot > 0:
                    self.decided = True
                    self.selected_slot = slot
                    self.selected_class = cls_id
                    self.final_layout = layout
                    layout_str = ','.join(f'{s}:{self.class_names.get(c, "?")}' for s, c in layout)
                    self.get_logger().info(
                        f'DECIDED layout=[{layout_str}] -> pick {self.class_names.get(cls_id, "?")} at slot {slot}'
                    )
                    pmsg = Int32(); pmsg.data = int(slot); self.position_pub.publish(pmsg)
                    smsg = String(); smsg.data = layout_str; self.layout_pub.publish(smsg)
                    self._send_slot(slot)
            else:
                seen_unique = len(set(self.history))
                cv2.putText(frame, f"Stab: {len(self.history)}/{stable_frames} uniq={seen_unique}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            if self.frame_count % 30 == 0:
                pmsg = Int32(); pmsg.data = int(self.selected_slot); self.position_pub.publish(pmsg)
            cv2.putText(frame,
                        f"DECIDED: slot {self.selected_slot} ({self.class_names.get(self.selected_class, '?')})",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if len(detections_data) > 0:
            msg = Float32MultiArray()
            msg.data = detections_data
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Detected {len(detections_data)//3} objects | FPS: {fps:.1f} | decided={self.decided}'
            )
            if self.frame_count % self.show_every_n == 0:
                cv2.imwrite('/home/minmin/roboarm_ws/yolo_select_debug.png', frame)
        else:
            self.get_logger().info(f'Searching... | FPS: {fps:.1f}', throttle_duration_sec=1.0)

        if self.frame_count % self.show_every_n == 0:
            cv2.imwrite('/home/minmin/roboarm_ws/camera_test.png', frame)
            cv2.imshow('Robot Vision (Select)', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloSelectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
