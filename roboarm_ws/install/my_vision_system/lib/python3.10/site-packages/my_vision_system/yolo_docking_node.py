import math
import os
from enum import Enum

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from ultralytics import YOLO


class DockState(Enum):
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    FINAL_ALIGN = 'FINAL_ALIGN'
    DOCKED = 'DOCKED'


class YoloDockingNode(Node):
    def __init__(self):
        super().__init__('yolo_docking_node')

        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('target_class', 2)
        self.declare_parameter('conf_threshold', 0.5)

        self.declare_parameter('approach_distance', 0.60)
        self.declare_parameter('docking_distance', 0.40)
        self.declare_parameter('safety_stop_dist', 0.20)

        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('ki_linear', 0.3)
        self.declare_parameter('kp_lateral', 0.7)
        self.declare_parameter('kp_angular_pixel', 1.0)
        self.declare_parameter('kp_angular_edge', 1.4)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_lateral', 0.25)
        self.declare_parameter('max_angular', 0.5)

        self.declare_parameter('dist_deadband', 0.03)
        self.declare_parameter('errx_deadband', 0.05)
        self.declare_parameter('angle_deadband_rad', 0.035)
        self.declare_parameter('docked_frames_required', 5)

        self.declare_parameter('detection_hold_sec', 0.6)
        self.declare_parameter('lost_timeout_sec', 2.0)

        self.declare_parameter('hfov', 1.047)
        self.declare_parameter('search_angular_speed', 0.3)

        self.declare_parameter('tilt_sign', -1)
        self.declare_parameter('ema_alpha', 0.4)

        model_name = self.get_parameter('model_path').value
        try:
            share_dir = get_package_share_directory('my_vision_system')
            model_path = os.path.join(share_dir, 'models', model_name)
            if not os.path.exists(model_path):
                curr_dir = os.path.dirname(os.path.abspath(__file__))
                model_path = os.path.join(curr_dir, 'models', model_name)
        except Exception:
            curr_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(curr_dir, 'models', model_name)

        self.get_logger().info(f'Loading YOLO: {model_path}')
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.get_logger().info(f'YOLO device: {self.device}, class names: {self.model.names}')

        self.bridge = CvBridge()
        self.state = DockState.SEARCH
        self.current_frame = None
        self.last_detection = None
        self.last_detection_time = self.get_clock().now()
        self.min_front_range = float('inf')
        self.linear_integral = 0.0
        self.last_control_time = self.get_clock().now()
        self.docked_counter = 0
        self.sm_cx = None
        self.sm_angle = None

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        self.status_pub = self.create_publisher(Bool, '/docking_status', 10)
        self.state_pub = self.create_publisher(String, '/docking_state', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('YoloDockingNode (spear) initialized')

    def image_callback(self, msg):
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        target_cls = self.get_parameter('target_class').value
        conf_th = self.get_parameter('conf_threshold').value
        results = self.model(self.current_frame, verbose=False, conf=conf_th)

        best = None
        best_conf = 0.0
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if cls_id != target_cls:
                    continue
                conf = float(box.conf[0])
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                    best = {
                        'cx': (x1 + x2) / 2.0,
                        'cy': (y1 + y2) / 2.0,
                        'bw': x2 - x1,
                        'bh': y2 - y1,
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'conf': conf,
                    }

        if best is None:
            return

        base_angle, base_edge = self.detect_base_top_edge(
            self.current_frame, best['bbox'])
        best['base_angle'] = base_angle
        best['base_edge'] = base_edge

        alpha = self.get_parameter('ema_alpha').value
        self.sm_cx = best['cx'] if self.sm_cx is None else alpha * best['cx'] + (1 - alpha) * self.sm_cx
        if base_angle is not None:
            self.sm_angle = base_angle if self.sm_angle is None else alpha * base_angle + (1 - alpha) * self.sm_angle
        best['sm_cx'] = self.sm_cx
        best['sm_angle'] = self.sm_angle

        self.last_detection = best
        self.last_detection_time = self.get_clock().now()

    def detect_base_top_edge(self, frame, spear_bbox):
        x1, y1, x2, y2 = spear_bbox
        bh = y2 - y1
        h, w = frame.shape[:2]

        sy1 = max(0, y2 - int(bh * 0.1))
        sy2 = min(h, y2 + int(max(bh * 1.2, 40)))
        if sy2 - sy1 < 8:
            return None, None

        roi = frame[sy1:sy2, 0:w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 40, 120)

        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=35,
            minLineLength=int(w * 0.18), maxLineGap=20,
        )
        if lines is None:
            return None, None

        best_line = None
        best_length = 0
        for L in lines:
            lx1, ly1, lx2, ly2 = L[0]
            dx = lx2 - lx1
            dy = ly2 - ly1
            if dx == 0:
                continue
            if dx < 0:
                lx1, lx2 = lx2, lx1
                ly1, ly2 = ly2, ly1
                dx = -dx
                dy = -dy
            angle = math.atan2(dy, dx)
            if abs(angle) > math.radians(30):
                continue
            length = math.hypot(dx, dy)
            if length > best_length:
                best_length = length
                best_line = (lx1, ly1 + sy1, lx2, ly2 + sy1, angle)

        if best_line is None:
            return None, None
        bx1, by1, bx2, by2, angle = best_line
        return angle, ((bx1, by1), (bx2, by2))

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        front_mask = np.abs(angles) < math.radians(20)
        front = ranges[front_mask]
        valid = front[np.isfinite(front) & (front > msg.range_min)]
        self.min_front_range = float(np.min(valid)) if len(valid) > 0 else float('inf')

    def have_detection(self):
        if self.last_detection is None:
            return False
        hold = self.get_parameter('detection_hold_sec').value
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return elapsed < hold

    def is_lost(self):
        if self.last_detection is None:
            return True
        timeout = self.get_parameter('lost_timeout_sec').value
        elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        return elapsed > timeout

    def control_loop(self):
        if self.current_frame is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        dt = max(min(dt, 0.1), 1e-3)

        s = String()
        s.data = self.state.value
        self.state_pub.publish(s)

        if self.state == DockState.SEARCH:
            if self.have_detection():
                self.state = DockState.APPROACH
                self.linear_integral = 0.0
                self.get_logger().info('SEARCH -> APPROACH')
        elif self.state == DockState.APPROACH:
            if self.is_lost():
                self._reset_to_search('APPROACH -> SEARCH (lost)')
            else:
                approach_d = self.get_parameter('approach_distance').value
                if math.isfinite(self.min_front_range) and self.min_front_range < approach_d:
                    self.state = DockState.FINAL_ALIGN
                    self.get_logger().info('APPROACH -> FINAL_ALIGN')
        elif self.state == DockState.FINAL_ALIGN:
            if self.is_lost():
                self._reset_to_search('FINAL_ALIGN -> SEARCH (lost)')

        debug = self.current_frame.copy()
        if self.state == DockState.SEARCH:
            cmd = self._act_search(debug)
        elif self.state == DockState.APPROACH:
            cmd = self._act_approach(debug, dt)
        elif self.state == DockState.FINAL_ALIGN:
            cmd = self._act_final_align(debug, dt)
        else:
            cmd = Twist()
            self._publish_docked(debug)

        if self.state != DockState.DOCKED:
            safety = self.get_parameter('safety_stop_dist').value
            if math.isfinite(self.min_front_range) and self.min_front_range < safety:
                if cmd.linear.x > 0.0:
                    cmd.linear.x = 0.0
                cv2.putText(debug, 'SAFETY', (50, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        self.cmd_pub.publish(cmd)
        self._draw_overlay(debug, cmd)
        small = cv2.resize(debug, (320, 320))
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(small, encoding='bgr8'))

    def _reset_to_search(self, reason):
        self.state = DockState.SEARCH
        self.docked_counter = 0
        self.linear_integral = 0.0
        self.sm_cx = None
        self.sm_angle = None
        self.get_logger().info(reason)

    def _act_search(self, debug):
        cmd = Twist()
        cmd.angular.z = self.get_parameter('search_angular_speed').value
        cv2.putText(debug, 'SEARCH', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        self.docked_counter = 0
        return cmd

    def _act_approach(self, debug, dt):
        cmd = Twist()
        d = self.last_detection
        img_w = self.current_frame.shape[1]
        hfov = self.get_parameter('hfov').value

        cx = d.get('sm_cx', d['cx'])
        err_x_norm = (cx - img_w / 2.0) / (img_w / 2.0)
        angle_pixel = err_x_norm * (hfov / 2.0)

        kp_a = self.get_parameter('kp_angular_pixel').value
        max_a = self.get_parameter('max_angular').value
        cmd.angular.z = float(np.clip(-kp_a * angle_pixel, -max_a, max_a))

        kp_lat = self.get_parameter('kp_lateral').value * 0.5
        max_lat = self.get_parameter('max_lateral').value
        cmd.linear.y = float(np.clip(-kp_lat * err_x_norm, -max_lat, max_lat))

        approach_d = self.get_parameter('approach_distance').value
        dist = self.min_front_range
        max_l = self.get_parameter('max_linear').value
        kp_l = self.get_parameter('kp_linear').value
        if math.isfinite(dist):
            err_d = dist - approach_d
            cmd.linear.x = float(np.clip(kp_l * err_d, -max_l, max_l))
        else:
            cmd.linear.x = 0.15

        cv2.putText(debug, 'APPROACH', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return cmd

    def _act_final_align(self, debug, dt):
        cmd = Twist()
        d = self.last_detection
        img_w = self.current_frame.shape[1]

        cx = d.get('sm_cx', d['cx'])
        err_x_norm = (cx - img_w / 2.0) / (img_w / 2.0)

        base_angle = d.get('sm_angle')
        if base_angle is None:
            base_angle = d.get('base_angle')

        kp_lat = self.get_parameter('kp_lateral').value
        max_lat = self.get_parameter('max_lateral').value
        cmd.linear.y = float(np.clip(-kp_lat * err_x_norm, -max_lat, max_lat))

        max_a = self.get_parameter('max_angular').value
        if base_angle is not None:
            kp_a = self.get_parameter('kp_angular_edge').value
            tilt_sign = self.get_parameter('tilt_sign').value
            cmd.angular.z = float(np.clip(kp_a * tilt_sign * base_angle, -max_a, max_a))
            angle_ok = abs(base_angle) < self.get_parameter('angle_deadband_rad').value
        else:
            hfov = self.get_parameter('hfov').value
            angle_pixel = err_x_norm * (hfov / 2.0)
            kp_a = self.get_parameter('kp_angular_pixel').value
            cmd.angular.z = float(np.clip(-kp_a * angle_pixel, -max_a, max_a))
            angle_ok = abs(angle_pixel) < self.get_parameter('angle_deadband_rad').value

        dock_d = self.get_parameter('docking_distance').value
        dist = self.min_front_range
        kp_l = self.get_parameter('kp_linear').value
        ki_l = self.get_parameter('ki_linear').value
        max_l = self.get_parameter('max_linear').value * 0.7

        dist_ok = False
        if math.isfinite(dist):
            err_d = dist - dock_d
            raw = kp_l * err_d + ki_l * self.linear_integral
            if abs(raw) < max_l:
                self.linear_integral += err_d * dt
            cmd.linear.x = float(np.clip(kp_l * err_d + ki_l * self.linear_integral,
                                         -max_l, max_l))
            dist_ok = abs(err_d) < self.get_parameter('dist_deadband').value
        else:
            self.linear_integral = 0.0
            cmd.linear.x = 0.05

        errx_ok = abs(err_x_norm) < self.get_parameter('errx_deadband').value

        if dist_ok and errx_ok and angle_ok:
            self.docked_counter += 1
        else:
            self.docked_counter = 0

        need = self.get_parameter('docked_frames_required').value
        if self.docked_counter >= need:
            self.state = DockState.DOCKED
            self.get_logger().info('FINAL_ALIGN -> DOCKED')
            cmd = Twist()

        cv2.putText(debug, 'FINAL_ALIGN', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        ang_str = f'{base_angle:+.3f}' if base_angle is not None else 'N/A'
        cv2.putText(debug,
                    f'd={dist:.2f} ex={err_x_norm:+.2f} a={ang_str} ok[{int(dist_ok)}{int(errx_ok)}{int(angle_ok)}] n={self.docked_counter}',
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)
        return cmd

    def _publish_docked(self, debug):
        cv2.putText(debug, 'DOCKED', (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
        msg = Bool()
        msg.data = True
        self.status_pub.publish(msg)

    def _draw_overlay(self, debug, cmd):
        h, w = debug.shape[:2]
        cv2.line(debug, (w // 2, 0), (w // 2, h), (128, 128, 128), 1)

        if self.last_detection is not None:
            d = self.last_detection
            x1, y1, x2, y2 = d['bbox']
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(debug, f'spear {d["conf"]:.2f}', (x1, max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.circle(debug, (int(d['cx']), int(d['cy'])), 4, (0, 255, 0), -1)
            edge = d.get('base_edge')
            if edge is not None:
                p1, p2 = edge
                cv2.line(debug, p1, p2, (255, 0, 255), 2)

        cv2.putText(debug, f'vx={cmd.linear.x:+.2f} vy={cmd.linear.y:+.2f} wz={cmd.angular.z:+.2f}',
                    (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDockingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    try:
        node.cmd_pub.publish(Twist())
    except Exception:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
