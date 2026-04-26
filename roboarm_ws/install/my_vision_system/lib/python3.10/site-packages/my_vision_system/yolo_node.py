import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import glob
import os
import torch
from ultralytics import YOLO
from collections import deque

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.publisher_ = self.create_publisher(Float32MultiArray, '/detected_object', 10)

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
        self.declare_parameter('cap_fps', 60)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.8)
        self.declare_parameter('grayscale', True)
        self.declare_parameter('expected_slots', 3)
        self.declare_parameter('priority_order', [0, 1, 2])
        self.declare_parameter('stable_frames', 20)

        self.cap = None
        self.cam_path = None
        self._open_camera()

        self.class_names = {0: 'PAPER', 1: 'ROCK', 2: 'SPEAR'}
        self.prev_time = self.get_clock().now()

        sf = self.get_parameter('stable_frames').value
        self.history = deque(maxlen=sf)
        self.decided = False
        self.last_decision_text = ""
        self.last_decision_color = (0, 255, 0)
        self.missed_frames = 0

        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('YOLO Node พร้อมทำงาน! (Every-frame inference, no skip)')

    def _resolve_camera_path(self):
        explicit = self.get_parameter('camera_path').get_parameter_value().string_value
        if explicit and os.path.exists(explicit):
            return explicit
        match = self.get_parameter('camera_match').get_parameter_value().string_value
        candidates = sorted(glob.glob('/dev/v4l/by-id/*'))
        for link in candidates:
            if match.lower() in os.path.basename(link).lower() and 'index0' in link:
                return os.path.realpath(link)
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

    def _assign_slots(self, dets_sorted):
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
                if cls_id not in cls_to_slots:
                    cls_to_slots[cls_id] = []
                cls_to_slots[cls_id].append(slot)
            if p_cls in cls_to_slots:
                return (min(cls_to_slots[p_cls]), p_cls)
        return (-1, -1)

    def _run_loop(self):
        while self._running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                self._open_camera()
                continue

            ret, frame = self.cap.read()
            if not ret:
                self._open_camera()
                continue

            if self.get_parameter('grayscale').value:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            current_time = self.get_clock().now()
            diff = (current_time - self.prev_time).nanoseconds / 1e9
            fps = 1.0 / diff if diff > 0 else 0.0
            self.prev_time = current_time

            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            results = self.model.predict(
                frame,
                device=self.device,
                imgsz=self.get_parameter('imgsz').value,
                conf=self.get_parameter('conf_thresh').value,
                iou=0.45,
                agnostic_nms=True,
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
                    dets.append((cx, cy, cls_id))

                    class_name = self.class_names.get(cls_id, 'UNKNOWN')
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 2)
                    cv2.putText(frame, f"{class_name} {conf:.2f}", (int(x1), int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            if len(dets) > 0:
                self.missed_frames = 0
                sorted_dets = sorted(dets, key=lambda d: d[0])
                simple = [(d[0], d[2]) for d in sorted_dets]
                layout = self._assign_slots(simple)
                sig = tuple(layout)
                self.history.append(sig)
            else:
                self.missed_frames += 1
                if self.missed_frames > 10:
                    self.history.clear()
                    self.decided = False
                    self.last_decision_text = "SEARCHING..."
                    self.last_decision_color = (0, 0, 255)

            stable_frames = self.get_parameter('stable_frames').value
            if len(self.history) >= 5:
                counts = {}
                for s in self.history:
                    counts[s] = counts.get(s, 0) + 1

                most_common_sig = max(counts, key=counts.get)
                frequency = counts[most_common_sig]

                if frequency >= (len(self.history) * 0.75) and len(self.history) >= 10:
                    if not self.decided:
                        layout_list = list(most_common_sig)
                        priority = list(self.get_parameter('priority_order').value)
                        slot, cls_id = self._decide(layout_list, priority)

                        if slot > 0:
                            self.decided = True
                            class_to_cmd = {0: 'C', 1: 'B', 2: 'A'}
                            cmd = class_to_cmd.get(cls_id, 'A')

                            all_spears = all(d[1] == 2 for d in layout_list)
                            if all_spears and cls_id == 2:
                                self.last_decision_text = f"DECISION: {cmd} (SPEAR ONLY)"
                                self.last_decision_color = (0, 255, 255)
                            else:
                                self.last_decision_text = f"DECISION: {cmd} ({self.class_names.get(cls_id)})"
                                self.last_decision_color = (0, 255, 0)

                    pct = int(frequency / len(self.history) * 100)
                    current_layout_str = " | ".join(
                        [f"S{s}:{self.class_names.get(c, '??')}" for s, c in list(most_common_sig)])
                    cv2.putText(frame, f"Match: {current_layout_str} ({pct}%)", (10, frame.shape[0]-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    if not self.decided:
                        self.last_decision_text = f"COLLECTING DATA: {len(self.history)}/{stable_frames}"
                        self.last_decision_color = (0, 165, 255)

            cv2.putText(frame, self.last_decision_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.last_decision_color, 2)

            if len(detections_data) > 0:
                msg = Float32MultiArray()
                msg.data = detections_data
                self.publisher_.publish(msg)
            else:
                self.get_logger().info(f'Searching... | FPS: {fps:.1f}', throttle_duration_sec=1.0)

            cv2.imshow('Robot Vision', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
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
